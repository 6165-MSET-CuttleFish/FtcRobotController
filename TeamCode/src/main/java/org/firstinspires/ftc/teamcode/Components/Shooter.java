package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.TuningController;
import org.firstinspires.ftc.teamcode.util.VelocityPIDFController;

import java.util.Arrays;
import java.util.HashSet;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;
import static org.firstinspires.ftc.teamcode.util.TuningController.MOTOR_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.util.TuningController.MOTOR_TICKS_PER_REV;

//http://192.168.43.1:8080/dash
@Config
public class Shooter implements Component {
    public enum State {
        CONTINUOUS,
        CUSTOMVELO,
        POWERSHOTS,
        TUNING,
        IDLE,
    }

    State state = State.IDLE;
    public StateMachine powerShotsController;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.002, 0, 0);
    public static double kV = 0.000162;//1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.00027;
    public static double kStatic = 0;
    public static double threshold = 300;

    double lastTargetVelo = 0.0;
    double lastMotorPos = 0;
    double lastMotorVelo = 0;
    double lastAccel = 0;
    double lastKv = kV;
    double lastKa = kA;
    double lastKstatic = kStatic;
    VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
    TuningController tuner;
    VoltageSensor batteryVoltageSensor;
    private final ElapsedTime veloTimer = new ElapsedTime();
    private final ElapsedTime coastTimer = new ElapsedTime();
    public DcMotor flywheel, flywheel1;
    public Encoder veloTracker;
    public Servo flap;
    public double targetVelo;
    public double power;
    private final InterpLUT veloRegression;
    public ColorRangeSensor colorRangeSensor;
    public Magazine magazine;
    public Gunner gunner;
    public Turret turret;
    HashSet<Component> components;

    public Shooter(HardwareMap hardwareMap) {
        veloRegression = new InterpLUT();
        setVelocityController();
        flywheel = hardwareMap.get(DcMotor.class, "fw");
        flywheel1 = hardwareMap.get(DcMotor.class, "fw1");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        veloTracker = new Encoder(hardwareMap.get(DcMotorEx.class, "fw1"));
        veloTracker.setDirection(Encoder.Direction.REVERSE);


        magazine = new Magazine(hardwareMap);
        gunner = new Gunner(hardwareMap);
        turret = new Turret(hardwareMap);
        components = new HashSet<>(Arrays.asList(magazine, gunner, turret));
        flap = hardwareMap.get(Servo.class, "flap");

        if (Details.opModeType == OpModeType.AUTO) {
            flapWayDown();
        } else {
            flapDown();
        }
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        powerShotsController = new StateMachineBuilder<Integer>()
                .state(0)
                .transition(() -> turret.isIdle())
                .onEnter(() -> turret.setTarget(Robot.powerShots[0]))

                .state(1)
                .transition(() -> gunner.getState() != Gunner.State.IN)
                .onEnter(() -> gunner.shoot(1))

                .state(2)
                .transition(() -> turret.isIdle())
                .onEnter(() -> turret.setTarget(Robot.powerShots[1]))

                .state(3)
                .transition(() -> gunner.getState() != Gunner.State.IN)
                .onEnter(() -> gunner.shoot(1))

                .state(4)
                .transition(() -> turret.isIdle())
                .onEnter(() -> turret.setTarget(Robot.powerShots[2]))

                .state(5)
                .transition(() -> gunner.getState() != Gunner.State.IN)
                .onEnter(() -> gunner.shoot(1))

                .exit(0)

                .build();
        tuner = new TuningController();
        veloTimer.reset();
        coastTimer.reset();
        setPIDCoeffecients();
    }

    public void update() {
        for (Component component : components) {
            component.update();
        }
        switch (state) {
            case CONTINUOUS:
                targetVelo = veloRegression.get(Details.robotPose.vec().distTo(Robot.goal));
                break;
            case POWERSHOTS:
                targetVelo = 1000;
                powerShotsController.update();
                break;
            case TUNING:
                if(!tuner.getRunning()) tuner.start();
                targetVelo = tuner.update();
                break;
            case IDLE:
                targetVelo = 0;
                break;
        }
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();
        lastTargetVelo = targetVelo;
        double motorPos = veloTracker.getCurrentPosition();
        double motorVelo = (veloTracker.getCorrectedVelocity()/8192)*60;
        double accel = motorVelo - lastMotorVelo;
        double power;
        if(Math.abs(accel - lastAccel) < threshold) {
            power = veloController.update(motorPos, motorVelo);
            lastMotorVelo = motorVelo;
            lastMotorPos = motorPos;
            packet.put("Shooter Velocity", motorVelo);
            packet.put("Shooter Accel", accel);
        }
        else {
            power = veloController.update(lastMotorPos, lastMotorVelo);
            packet.put("Shooter Velocity", lastMotorVelo);
            packet.put("Shooter Accel", lastAccel);
        }
        lastAccel = accel;
        if (targetVelo == 0) {
            flywheel.setPower(this.power);
            flywheel1.setPower(this.power);
        } else {
            flywheel.setPower(power);
            flywheel1.setPower(power);
        }
        if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
            lastKv = kV;
            lastKa = kA;
            lastKstatic = kStatic;
            veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        }
        setPIDCoeffecients();
        packet.put("Target Velocity", targetVelo);
        packet.put("Motor Power", power);
        //dashboard.sendTelemetryPacket(packet);
    }

    private void setPIDCoeffecients() {
        veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV * 12 / batteryVoltageSensor.getVoltage());
    }

    public int getRings() {
        return 0;
    }

    public double getDistance() {
        return colorRangeSensor.getDistance(DistanceUnit.INCH);
    }

    public void setVelocity(double v) {
        targetVelo = v;
    }

    public void setVelocity(Vector2d vector2d) {
        targetVelo = veloRegression.get(vector2d.distTo(Robot.goal));
    }

    public double getPoseVelo(Vector2d vector2d) {
        return veloRegression.get(vector2d.distTo(Robot.goal));
    }

    public double getVelocity() {
        return 0;
        //return veloTracker.getCorrectedVelocity();
    }

    public double getTargetVelo() {
        return targetVelo;
    }

    public double getError() {
        return getTargetVelo() - getVelocity();
    }

    public double getAbsError() {
        return Math.abs(getError());
    }

    public double getPercentError() {
        return getAbsError() / getTargetVelo();
    }

    public void tripleShot() {
        gunner.shoot(3);
    }


    public void flapUp() {
        flap.setPosition(0.75);
    }

    public void flapDown() {
        flap.setPosition(0.93);
    }

    public void flapWayDown() {
        flap.setPosition(1);
    }

    private void setVelocityController() {
        veloRegression.add(0, 1500);
        veloRegression.add(75, 1450);
        veloRegression.add(77.5, 1440);
        veloRegression.add(78, 1420);
        veloRegression.add(79, 1410);
        veloRegression.add(80, 1400);
        veloRegression.add(85, 1380);
        veloRegression.add(90, 1350);
        veloRegression.add(95, 1230);
        veloRegression.add(100, 1220);
        veloRegression.add(102, 1210);
        veloRegression.add(105, 1220);
        veloRegression.add(110, 1220);
        veloRegression.add(115, 1190);
        veloRegression.add(120, 1190);
        veloRegression.add(125, 1210);
        veloRegression.add(132.5, 1220);
        veloRegression.add(200, 1400);
        veloRegression.add(1000, 1400);
        veloRegression.createLUT();
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void powerShots() {
        powerShotsController.start();
    }
}