package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.modules.wrappers.Linkage;
import org.firstinspires.ftc.teamcode.modules.wrappers.V4B;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableMotor;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;
import org.firstinspires.ftc.teamcode.util.controllers.BPIDFController;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Ayush Raman
 */
@Config
public class Deposit extends Module<Deposit.State> {
    public static double outPosition3 = 0.32;
    public static double outPosition2 = 0.23;
    public static double outPosition1 = 0.23;
    public double offsetOutPosition;
    public static double
            outOffsetPower,
            outOffsetIncrement = 0.03;
    public static double extendIn = 0.26, extendOut = 0.0;
    public double offsetExtendPosition;
    public static double
            linkageOffsetPower,
            linkageOffsetIncrement = 0.1;
    public static double holdingPosition = 0.4;
    public static double
            inPosition = 0.5,
            higherInPosition = 0.47;
    public static double
            lockPosition = 0.72,
            unlockPosition = 0.56,
            kickPosition = 0.95;
    public static double
            armServoPositionPerSecond = 0.5,
            extensionServoPositionPerSecond = 0.6;
    public static boolean isLoaded;
    public boolean shouldCounterBalance = true;
    public static double dumpTimeOut = 0.13;
    private boolean allowLift = false;
    private boolean farDeposit = false;
    @Override
    public boolean isTransitioningState() {
        return extension.isTransitioning() || arm.isTransitioning() || Math.abs(getLastError()) > allowableDepositError;
    }

    public enum State implements StateBuilder {
        IN(0.5),
        CREATE_CLEARANCE,
        LOCKING(0.16),
        HOLDING,
        DUMPING(0.15),
        SOFT_DUMP(0.1),
        OUT1,
        OUT2,
        OUT3;
        private final Double timeOut;
        @Override
        public Double getTimeOut() {
            return this.timeOut;
        }
        State(Double timeOut) {
            this.timeOut = timeOut;
        }
        State() {
            this.timeOut = null;
        }
    }

    public enum Level {
        LEVEL3,
        LEVEL2,
        LEVEL1,
        IDLE
    }

    private ControllableServos lock;
    private V4B arm;
    private Linkage extension;
    private final Intake intake;
    private ControllableMotor slides;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0.7,0,0.01);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    private BPIDFController pidController = new BPIDFController(MOTOR_PID, kV, kA, kStatic);

    double lastKv = kV;
    double lastKa = kA;
    double lastKStatic = kStatic;
    double lastKp = MOTOR_PID.kP;
    double lastKi = MOTOR_PID.kI;
    double lastKd = MOTOR_PID.kD;
    public static double TICKS_PER_INCH = 61.74;
    boolean intakeCleared;
    private Level defaultLevel = Level.LEVEL3;


    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap, Intake intake) {
        super(hardwareMap, opModeType == OpModeType.AUTO ? State.LOCKING : State.IN, new Pose2d());
        this.intake = intake;
    }

    public void setLevel(Level level) {
        if (level == defaultLevel) return;
        if (level == Level.IDLE) defaultLevel = Level.LEVEL1;
        else defaultLevel = level;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        Servo
                dumpLeft = hardwareMap.servo.get("depositDumpL");
        arm = new V4B(7.09, new ControllableServos(dumpLeft));
        Servo
                extL = hardwareMap.servo.get("extL"),
                extR = hardwareMap.servo.get("extR");
        extR.setDirection(Servo.Direction.REVERSE);
        extension = new Linkage(9.961, 4.40945, 6.2795276, new ControllableServos(extL, extR));
        lock = new ControllableServos(hardwareMap.servo.get("lock"));
        unlock();
        if (opModeType == OpModeType.AUTO) lock();
        slides = new ControllableMotor(hardwareMap.get(DcMotorEx.class, "depositSlides"));
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipIn();
        arm.setPosition(inPosition);
        setActuators(lock, slides);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        arm.getServos().setPositionPerSecond(armServoPositionPerSecond);
        extension.getServos().setPositionPerSecond(extensionServoPositionPerSecond);
        extension.getServos().setLimits(extendOut, extendIn);
        switch (getState()) {
            case IN:
                offsetOutPosition = 0.0;
                offsetExtendPosition = 0.0;
                if (!Deposit.isLoaded) unlock();
                flipIn();
                if (!extension.isTransitioning()) {
                    pidController.setTargetPosition(getLevelHeight(getLevel()));
                }
                if (isLoaded) {
                    setState(State.LOCKING);
                }
                if (!intakeCleared && !isTransitioningState()) {
                    if (intake.getState() != Intake.State.OUT && intake.getState() != Intake.State.TRANSFER) intake.retractIntake();
                    intakeCleared = true;
                }
                if (intake.getState() == Intake.State.OUT && intake.isTransitioningState() && !extension.isTransitioning() && Math.abs(getLastError()) > allowableDepositError) {
                    setState(State.CREATE_CLEARANCE);
                }
                break;
            case CREATE_CLEARANCE:
                arm.setPosition(higherInPosition);
                if (intake.getState() != Intake.State.OUT) {
                    setState(State.IN);
                }
                break;
            case LOCKING:
                lock();
                if (getSecondsSpentInState() > getState().timeOut) {
                    setState(State.HOLDING);
                }
                break;
            case HOLDING:
                holdingPosition();
                pidController.setTargetPosition(getLevelHeight(getLevel()));
                if (allowLift) {
                    prepPlatform(getLevel());
                }
                break;
            case OUT1:
            case OUT2:
            case OUT3:
                pidController.setTargetPosition(getLevelHeight(getLevel()));
                if (getState() == State.OUT1 || !shouldCounterBalance) {
                    intake.retractIntake();
                } else {
                    intake.counterBalance();
                }
                intakeCleared = false;
                setState(getNeededOutState(getLevel()));
                if (!allowLift) {
                    flipIn();
                    setState(State.IN);
                }
                lock();
                flipOut(getLevel());
                break;
            case DUMPING:
            case SOFT_DUMP:
                double diff = kickPosition - lockPosition;
                if (getState() == State.SOFT_DUMP) lock.setPosition(Range.clip(lockPosition + diff * getSecondsSpentInState() / dumpTimeOut, lockPosition, kickPosition));
                else lock.setPosition(kickPosition);
                if (!allowLift) {
                    if (getPreviousState() == State.OUT1) {
                        intake.createClearance();
                    }
                    setState(State.IN);
                }
                if (getSecondsSpentInState() > dumpTimeOut + getState().timeOut) {
                    allowLift = false;
                    if (getPreviousState() == State.OUT1) {
                        intake.createClearance();
                    }
                    setState(State.IN);
                    isLoaded = false;
                }
                break;
        }
        double power = pidController.update(ticksToInches(slides.getCurrentPosition()));
        slides.setPower(power);
        // for dashboard
        if (kV != lastKv || kA != lastKa || kStatic != lastKStatic || MOTOR_PID.kP != lastKp || MOTOR_PID.kI != lastKi || MOTOR_PID.kD != lastKd) {
            lastKv = kV;
            lastKa = kA;
            lastKStatic = kStatic;
            lastKp = MOTOR_PID.kP;
            lastKi = MOTOR_PID.kI;
            lastKd = MOTOR_PID.kD;
            pidController = new BPIDFController(MOTOR_PID, kV, kA, kStatic);
        }
        if (isDebugMode()) {
            Context.packet.put("Lift Target Height", pidController.getTargetPosition());
            Context.packet.put("Lift Actual Height", ticksToInches(slides.getCurrentPosition()));
            Context.packet.put("Lift Motor Power", power);
            Context.packet.put("Lift Error", getLastError());
            Context.packet.put("isLoaded", isLoaded);
            Context.packet.put("Arm Real Height", arm.getVector().getY());
            Context.packet.put("Extension Real Distance", extension.getRealDisplacement());
        }
    }

    public void liftUp() {
        allowLift = true;
    }

    public void liftDown() {
        allowLift = false;
    }

    public void toggleLift() {
        allowLift = !allowLift;
    }

    public void setFarDeposit(boolean farDeposit) {
        this.farDeposit = farDeposit;
    }

    public void toggleFarDeposit() {
        farDeposit = !farDeposit;
    }

    public void moveArmPosition(double pwr) {
        offsetOutPosition =
                Range.clip(
                        offsetOutPosition + pwr * outOffsetPower * getMillisecondsSinceLastUpdate(),
                        arm.getServos().getLowerLimit(),
                        arm.getServos().getUpperLimit()
                );
    }

    public void incrementArmPosition(double pwr) {
        offsetOutPosition = Range.clip(
                offsetOutPosition + pwr * outOffsetIncrement,
                arm.getServos().getLowerLimit(),
                arm.getServos().getUpperLimit()
        );
    }

    public void moveLinkagePosition(double pwr) {
        offsetExtendPosition = Range.clip(
                offsetExtendPosition + pwr * linkageOffsetPower * getMillisecondsSinceLastUpdate(),
                extension.getServos().getLowerLimit(),
                extension.getServos().getUpperLimit()
        );
    }

    public void incrementLinkagePosition(double pwr) {
        offsetExtendPosition = Range.clip(
                offsetExtendPosition + pwr * linkageOffsetIncrement,
                extension.getServos().getLowerLimit(),
                extension.getServos().getUpperLimit()
        );
    }

    private Level getLevel() {
        if (getState() == State.IN) return Level.LEVEL1;
        return defaultLevel;
    }

    public static double weightSlides = 0.8, weightArm = 0.5, weightExtension = 0.3;

    public Vector2d getModuleWeightedVector() {
        Vector2d slidesVec = new Vector2d(
                ticksToInches(slides.getCurrentPosition()) * Math.cos(angle),
                ticksToInches(slides.getCurrentPosition()) * Math.sin(angle)
        );
        Vector2d extensionVec = new Vector2d().plus(slidesVec);
        if (platformIsOut()) {
            extensionVec = new Vector2d(13.4252).div(2).plus(slidesVec);
        }
        //Vector2d armVec = arm.getVector().plus(extensionVec);
        return ((slidesVec.times(weightSlides))
                .plus((extensionVec).times(weightExtension))
                //.plus(armVec.times(weightArm)))
                .div(weightSlides + weightExtension));
    }

    public double getWeightedDisplacement() {
        getModuleWeightedVector().component1();
        Vector2d vec = getModuleWeightedVector();
        return Math.hypot(vec.getX(), vec.getY());
    }

    public double getWeight() {
        return weightSlides + weightExtension;
    }

    public static double LEVEL3 = 12;
    public static double LEVEL2 = 3;
    public static double LEVEL1 = 0;
    public static double allowableDepositError = 2;
    public static double angle = Math.toRadians(30);

    private double getLevelHeight(Level state) {
        switch (state) {
            case LEVEL3: return LEVEL3;
            case LEVEL2: return LEVEL2;
            case LEVEL1: return LEVEL1;
        }
        return 0;
    }

    public double getLastError() {
        return pidController.getLastError();
    }

    /**
     * @param ticks current position of the motor
     * @return inches traveled by the slides
     */
    private static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     * @return servo position based on balance of hub
     */
    private double outPosition(Level state) {
        double outPos = outPosition3;
        switch (state) {
            case LEVEL3:
                outPos = outPosition3;
                break;
            case LEVEL2:
                outPos = outPosition2;
                break;
            case LEVEL1:
                outPos = outPosition1;
                break;
        }
        return outPos + offsetOutPosition;
    }

    /**
     * Extends the platform out
     */
    private void flipOut(Level state) {
        double position = outPosition(state);
        arm.setPosition(position);
        double extensionPos = extendOut;
        if (farDeposit) extensionPos  = extendIn / 2;
        extension.setPosition(extensionPos + offsetExtendPosition);
    }

    private void holdingPosition() {
        double position = holdingPosition;
        arm.setPosition(position);
    }

    /**
     * Return platform to rest
     */
    private void flipIn() {
        double position = inPosition;
        extension.setPosition(extendIn);
        if (!extension.isTransitioning() && Math.abs(ticksToInches(slides.getCurrentPosition()) - 0) < allowableDepositError) {
            arm.setPosition(position);
        } else {
            arm.setPosition(holdingPosition);
        }
    }

    /**
     * Dumps the loaded element onto hub
     */
    public void dump() {
        if (getState() == State.OUT3 || getState() == State.OUT2 || getState() == State.OUT1)
            setState(State.DUMPING);
    }

    /**
     * Dumps the loaded element onto hub
     */
    public void softDump() {
        if (getState() == State.OUT3 || getState() == State.OUT2 || getState() == State.OUT1)
            setState(State.SOFT_DUMP);
    }

    private State getNeededOutState(Level state) {
        switch (state) {
            case LEVEL3: return State.OUT3;
            case LEVEL2: return State.OUT2;
            case LEVEL1: return State.OUT1;
        }
        return State.OUT3;
    }

    /**
     * Puts platform into the prepped position
     */
    public void prepPlatform(Level state) {
        setState(getNeededOutState(state));
    }

    private void lock() {
        lock.setPosition(lockPosition);
    }

    private void unlock() {
        lock.setPosition(unlockPosition);
    }
  
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingInternalWork() {
        return getState() == State.DUMPING || platformIsOut() || getState() == State.SOFT_DUMP;
    }

    public boolean platformIsOut() {
        return getState() == State.OUT1 || getState() == State.OUT2 || getState() == State.OUT3;
    }
}
