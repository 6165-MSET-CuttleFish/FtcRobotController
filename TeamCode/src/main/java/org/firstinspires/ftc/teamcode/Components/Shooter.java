package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
        EMPTY_MAG,
        MID
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
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.001, 0, 0);
    public static double kV = 0.00017;
    public static double kA = 0.000027;
    public static double kStatic = 0.01;
    public static double threshold = 350;

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
    public DcMotor flywheel, flywheel1;
    public Encoder veloTracker;
    public Servo flap;
    public double targetVelo;
    private final InterpLUT veloRegression;
    private final InterpLUT toleranceRegression;
    public Magazine magazine;
    public Gunner gunner;
    public Turret turret;
    ElapsedTime timer = new ElapsedTime();
    private final Component[] components;

    public Shooter(HardwareMap hardwareMap) {
        veloRegression = new InterpLUT();
        toleranceRegression = new InterpLUT();
        setVelocityController();
        setToleranceRegression();
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
                .transition(() -> turret.isOnTarget() && gunner.getState() == Gunner.State.IDLE || timer.seconds() > 0.6)
                .onEnter(() -> {
                    timer.reset();
                    turret.setTarget(Robot.powerShots()[0]);
                })

                .state(PSState.PS1)
                .transitionTimed(4 * Gunner.gunTime)
                .onEnter(() -> gunner.shoot())

                .state(PSState.MOVING_PS2)
                .transition(() -> turret.isOnTarget() && gunner.getState() == Gunner.State.IDLE || timer.seconds() > 0.6)
                .onEnter(() -> {
                    turret.setTarget(Robot.powerShots()[1]);
                    timer.reset();
                })

                .state(PSState.PS2)
                .transitionTimed(4 * Gunner.gunTime)
                .onEnter(() -> gunner.shoot())

                .state(PSState.MOVING_PS3)
                .transition(() -> turret.isOnTarget() && gunner.getState() == Gunner.State.IDLE || timer.seconds() > 0.6)
                .onEnter(() -> {
                    timer.reset();
                    turret.setTarget(Robot.powerShots()[2]);
                })

                .state(PSState.PS3)
                .transitionTimed(4 * Gunner.gunTime)
                .onEnter(() -> gunner.shoot())

                .exit(PSState.MOVING_PS1)
                .onExit(() -> {
                    state = State.CONTINUOUS;
                    turret.setTarget(Robot.goal());
                })

                .build();
        tuner = new TuningController();
        veloTimer.reset();
        setPIDCoeffecients();
    }

    public void update() {
        for (Component component : components) {
            component.update();
        }
        Coordinate shooterCoord = Coordinate.toPoint(Details.robotPose).polarAdd(Details.robotPose.getHeading() - Math.PI, 3);
        switch (state) {
            case CONTINUOUS:
                flapUp();
                turret.setTarget(Robot.goal());
                try {
                    targetVelo = veloRegression.get(shooterCoord.distanceTo(Coordinate.toPoint(Robot.goal())));
                } catch (Exception e) {
                    targetVelo = 5000;
                }
                break;
            case POWERSHOTS:
                if (Details.opModeType == OpModeType.AUTO) {
                    targetVelo = 4000;
                    flapUp();
                } else {
                    targetVelo = 4200;
                    flapDown();
                }
                powerShotsController.update();
                break;
            /*case MID:
                flapUp();
                turret.setTarget(Robot.goal());
                targetVelo = 3800;
                break;
             */
            case TUNING:
                if(!tuner.getRunning()) tuner.start();
                targetVelo = tuner.update();
                break;
            case IDLE:
                targetVelo = 0;
                turret.setState(Turret.State.IDLE);
                break;
            case EMPTY_MAG:
                turret.setTargetAngle(Math.toRadians(-180));
                targetVelo = 2000;
                if (turret.isIdle() && Magazine.currentRings != 0 && turret.getAbsoluteAngle() < Math.toRadians(-100)) {
                    gunner.shoot();
                }
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
        return Coordinate.toPoint(Details.robotPose).polarAdd(Details.robotPose.getHeading() - Math.PI, 3).toVector();
    }

    public void setVelocity(double v) {
        setState(State.CONTINUOUS);
        targetVelo = v;
    }

    public void setVelocity(Vector2d vector2d) {
        targetVelo = veloRegression.get(vector2d.distTo(Robot.goal()));
    }

    public double getPoseVelo(Vector2d vector2d) {
        try {
            return veloRegression.get(vector2d.distTo(Robot.goal()));
        } catch (Exception ignored) {
            return 5700;
        }
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


    public void flapUp() {
        flap.setPosition(0.85);
    }

    public void flapWayUp() {
        flap.setPosition(0.82);
    }

    public void flapDown() {
        flap.setPosition(0.87);
    }

    public void flapWayDown() {
        flap.setPosition(1);
    }

    private void setVelocityController() {
        veloRegression.add(0,4500);
        veloRegression.add(67.7, 4520);
        veloRegression.add(72.8, 4520);
        veloRegression.add(78.4, 4600);
        veloRegression.add(82.23, 4630);
        veloRegression.add(86.3, 4670);
        veloRegression.add(88.4, 4670);
        veloRegression.add(95.2, 4850);
        veloRegression.add(110.5, 5000);
        veloRegression.add(115.8, 5100);
        veloRegression.add(136.5, 5700);
       
        veloRegression.createLUT();
    }

    private void setToleranceRegression() {
        toleranceRegression.add(0,200);
        toleranceRegression.add(67.7, 200);
        toleranceRegression.add(72.8, 190);
        toleranceRegression.add(78.4, 130);
        toleranceRegression.add(82.23, 100);
        toleranceRegression.add(86.3, 80);
        toleranceRegression.add(95.2, 80);

        toleranceRegression.createLUT();
    }

    public boolean isWithinTolerance() {
        double neededVelo = getPoseVelo(getShooterVec());
        double error = Math.abs(getVelocity() - neededVelo);
        double maxError;
        try {
            maxError = toleranceRegression.get(getShooterVec().distTo(Robot.goal()));
        } catch (Exception ignored) {
            maxError = 80;
        }
        return  maxError > error;
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