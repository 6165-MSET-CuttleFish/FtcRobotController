package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.TuningController;
import org.firstinspires.ftc.teamcode.util.VelocityPIDFController;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;

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
    public enum PSState {
        MOVING_PS1,
        PS1,
        MOVING_PS2,
        PS2,
        MOVING_PS3,
        PS3
    }

    State state = State.IDLE;
    public StateMachine powerShotsController;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0018, 0, 0.00001);
    public static double kV = 0.000168;
    public static double kA = 0.00003;
    public static double kStatic = 0.01;
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
    ElapsedTime timer = new ElapsedTime();
    VoltageSensor batteryVoltageSensor;
    private final ElapsedTime veloTimer = new ElapsedTime();
    public DcMotor flywheel, flywheel1;
    public Encoder veloTracker;
    public Servo flap;
    public double targetVelo;
    private final InterpLUT veloRegression;
    public ColorRangeSensor colorRangeSensor;
    public Magazine magazine;
    public Gunner gunner;
    public Turret turret;
    private final Component[] components;

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
        components = new Component[]{magazine, gunner, turret};
        flap = hardwareMap.get(Servo.class, "flap");

        if (Details.opModeType == OpModeType.AUTO) {
            flapWayDown();
        } else {
            flapDown();
        }
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        powerShotsController = new StateMachineBuilder<PSState>()
                .state(PSState.MOVING_PS1)
                .transition(() -> turret.isIdle() && gunner.getState() == Gunner.State.IDLE)
                .onEnter(() -> turret.setTarget(Robot.powerShots[0]))

                .state(PSState.PS1)
                .transition(() -> gunner.getState() == Gunner.State.IDLE)
                .onEnter(() -> gunner.shoot())

                .state(PSState.MOVING_PS2)
                .transition(() -> turret.isIdle() && gunner.getState() == Gunner.State.IDLE)
                .onEnter(() -> turret.setTarget(Robot.powerShots[1]))

                .state(PSState.PS2)
                .transition(() -> gunner.getState() == Gunner.State.IDLE)
                .onEnter(() -> gunner.shoot())

                .state(PSState.MOVING_PS3)
                .transition(() -> turret.isIdle() && gunner.getState() == Gunner.State.IDLE)
                .onEnter(() -> turret.setTarget(Robot.powerShots[2]))

                .state(PSState.PS3)
                .transition(() -> gunner.getState() == Gunner.State.IDLE)
                .onEnter(() -> gunner.shoot())

                .exit(PSState.MOVING_PS1)
                .onExit(() -> {
                    state = State.CONTINUOUS;
                    turret.setTarget(Robot.goal);
                })

                .build();
        tuner = new TuningController();
        veloTimer.reset();
        setPIDCoeffecients();
        timer.reset();
    }

    public void update() {
        for (Component component : components) {
            component.update();
        }
        Coordinate shooterCoord = Coordinate.toPoint(Details.robotPose).polarAdd(Details.robotPose.getHeading() - Math.PI, 4.5);
        switch (state) {
            case CONTINUOUS:
                flapUp();
                turret.setTarget(Robot.goal);
                try {
                    targetVelo = veloRegression.get(shooterCoord.distanceTo(Coordinate.toPoint(Robot.goal)));
                } catch (Exception e) {
                    targetVelo = 5000;
                }
                break;
            case POWERSHOTS:
                flapDown();
                targetVelo = 4200;
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
            flywheel.setPower(0);
            flywheel1.setPower(0);
        } else {
            flywheel.setPower(power);
            flywheel1.setPower(power);
        }
        if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
            lastKv = kV;
            lastKa = kA;
            lastKstatic = kStatic;
            timer.reset();
            setPIDCoeffecients();
            //veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV * 12 / batteryVoltageSensor.getVoltage(), kA, kStatic);
        }
        double newKv = kV * 12 / batteryVoltageSensor.getVoltage();
        veloController.setkV(newKv);
        packet.put("Target Velocity", targetVelo);
        packet.put("Motor Power", power);
    }

    private void setPIDCoeffecients() {
        veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
    }

    public Vector2d getShooterVec() {
        return Coordinate.toPoint(Details.robotPose).polarAdd(Details.robotPose.getHeading() - Math.PI, 4.5).toVector();
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
        return (veloTracker.getCorrectedVelocity()/8192)*60;
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
        flap.setPosition(0.85);
    }

    public void flapDown() {
        flap.setPosition(0.87);
    }

    public void flapWayDown() {
        flap.setPosition(1);
    }

    private void setVelocityController() {
        veloRegression.add(0,4400);
        veloRegression.add(67.7, 4500);
        veloRegression.add(72.8, 4500);
        veloRegression.add(78.4, 4600);
        veloRegression.add(82.23, 4500);
        veloRegression.add(86.3, 4670);
        veloRegression.add(88.4, 4670);
        veloRegression.add(95.2, 4850);
        veloRegression.add(110.5, 5000);
        veloRegression.add(115.8, 5100);
        veloRegression.add(136.5, 5700);
       
        veloRegression.createLUT();
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void powerShots() {
        if (!powerShotsController.getRunning()) powerShotsController.start();
    }
}