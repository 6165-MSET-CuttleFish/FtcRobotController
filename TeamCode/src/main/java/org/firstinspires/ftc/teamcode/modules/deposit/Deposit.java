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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.modules.wrappers.Linkage;
import org.firstinspires.ftc.teamcode.modules.wrappers.V4B;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableMotor;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;
import org.firstinspires.ftc.teamcode.util.controllers.BPIDFController;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.Freight;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.field.Side;

import static org.firstinspires.ftc.teamcode.util.field.Context.freight;
import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;
import static org.firstinspires.ftc.teamcode.util.field.Context.side;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Ayush Raman
 */
@Config
public class Deposit extends Module<Deposit.State> {
    public static double
            outPosition3 = 0.37,
            outPosition3WithLift = 0.28,
            outPosition2 = 0.19,
            outPosition1 = 0.0,
            outPositionShared = 0.15,
            outPositionSharedFar = 0.1;
    private double offsetOutPosition;
    public static double
            outOffsetPower,
            outOffsetIncrement = 0.05;
    public static double liftCurrentLimit = 5;
    public static double
            extendIn = 0.32,
            extendOut3 = 0.16,
            extendOut2 = 0.13,
            extendOut1 = 0.0,
            extendOutShared = 0.24,
            extendOutSharedFar = 0.15,
            extendTeleOffset = -0.03;
    private double offsetExtendPosition;
    public static double
            linkageOffsetPower,
            linkageOffsetIncrement = 0.1;
    public static double holdingPosition = 0.82;
    public static double
            inPosition = 0.97,
            higherInPosition = 0.9;
    public static double
            lockPosition = 0.68,
            unlockPosition = 0.52,
            kickPosition = 0.95,
            softKickPosition = 0.9;
    public static double
            armServoPositionPerSecond = 5.0,
            extensionServoPositionPerSecond = 0.8;
    public static boolean isLoaded;
    public boolean shouldCounterBalance = true;
    public static double dumpTimeOut = 0.2, softDumpTimeOut = 0.5;
    private boolean allowLift = false;
    @Override
    public boolean isTransitioningState() {
        return extension.isTransitioning() || arm.isTransitioning() || Math.abs(getLastError()) > allowableDepositError;
    }

    public enum State implements StateBuilder {
        IN(0.5),
        CREATE_CLEARANCE,
        LOCKING(0.15),
        HOLDING,
        DUMPING(0.1),
        SOFT_DUMP(0.1),
        OUT,
        RESETTING_ENCODER;
        private final double timeOut;
        @Override
        public Double getTimeOut() {
            return this.timeOut;
        }
        State(Double timeOut) {
            this.timeOut = timeOut;
        }
        State() {
            this.timeOut = 0.0;
        }
    }

    public enum Level {
        LEVEL3,
        LEVEL2,
        SHARED,
        LEVEL1,
    }

    public enum Distance {
        CLOSE,
        MEDIUM,
        FAR,
        CROSS,
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
    private Distance distance = Distance.CLOSE;


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
        defaultLevel = level;
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
        extension = new Linkage(9.961, 4.40945, 6.2795276, new ControllableServos(extL, extR), Math.toRadians(59.58));
        extension.getServos().setReversedAngle(true);
        extension.getServos().calibrateOffset(extendIn, Math.toRadians(59.58));
        lock = new ControllableServos(hardwareMap.servo.get("lock"));
        unlock();
        if (opModeType == OpModeType.AUTO) {
            lock();
        }
        else {
            unlock();
            lock();
            unlock();
            allowLift = true;
        }
        slides = new ControllableMotor(hardwareMap.get(DcMotorEx.class, "depositSlides"));
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipIn();
        arm.setPosition(holdingPosition);
        if (opModeType == OpModeType.AUTO) {
            arm.setPosition(holdingPosition);
            isLoaded = true;
        }
        setActuators(lock, slides);
    }
    public static double flipOutTime = 0.4;
    private int i = 0;
    private boolean toggle = false;
    public static boolean allowTransfer = true;
    private boolean farDeposit() {
        return distance == Distance.FAR;
    }

    private boolean closeDeposit() {
        return distance == Distance.CLOSE;
    }

    private boolean crossDeposit() {
        return distance == Distance.CROSS;
    }

    private boolean mediumDeposit() {
        return distance == Distance.MEDIUM;
    }


    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        i++;
        if (i >= 3) {
            toggle = !toggle;
        }
        arm.getServos().setPositionPerSecond(armServoPositionPerSecond);
        extension.getServos().setPositionPerSecond(extensionServoPositionPerSecond);
        extension.getServos().setLimits(0.0, extendIn);
        arm.getServos().setLimits(0.0, 1.0);
        switch (getState()) {
            case IN:
                if (!Deposit.isLoaded) unlock();
                flipIn();
                if (!extension.isTransitioning() || opModeType != OpModeType.TELE) {
                    pidController.setTargetPosition(0.0);
                }
                if (isLoaded) {
                    setState(State.LOCKING);
                }
                if (!intakeCleared && !isTransitioningState()) {
                    if (intake.getState() != Intake.State.OUT && intake.getState() != Intake.State.TRANSFER) intake.retractIntake();
                    intakeCleared = true;
                    allowTransfer = true;
                }
                if (intake.getState() == Intake.State.OUT && intake.isTransitioningState() && !extension.isTransitioning() && Math.abs(getLastError()) < allowableDepositError) {
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
                if (farDeposit()) pidController.setTargetPosition(getLevelHeight(getLevel()));
                else pidController.setTargetPosition(0.0);
                holdingPosition();
                if (allowLift) {
                    setState(State.OUT);
                }
                if (!Deposit.isLoaded) {
                    setState(State.IN);
                }
                break;
            case OUT:
                pidController.setTargetPosition(getLevelHeight(getLevel()));
                if (getLevel() == Level.SHARED || !shouldCounterBalance || farDeposit()) {
                    intake.retractIntake();
                } else {
                    intake.counterBalance();
                }
                intakeCleared = false;
                if (!allowLift) {
                    setState(State.HOLDING);
                }
                lock();
                flipOut(getLevel());
                break;
            case DUMPING:
            case SOFT_DUMP:
                if (getState() == State.SOFT_DUMP) {
                    double diff = softKickPosition - lockPosition;
                    if (getLevel() == Level.SHARED && !crossDeposit()) {
                        diff = 0 - lockPosition;
                        lock.setPosition(Range.clip(lockPosition + diff * getSecondsSpentInState() / softDumpTimeOut, 0.0, lockPosition));
                    } else {
                        lock.setPosition(Range.clip(lockPosition + diff * getSecondsSpentInState() / softDumpTimeOut, lockPosition, kickPosition));
                    }
                }
                else {
                    if (getLevel() == Level.SHARED && !crossDeposit()) lock.setPosition(0.0);
                    else lock.setPosition(kickPosition);
                }
                if (!allowLift) {
                    if (intake.getState() == Intake.State.IN) {
                        intake.createClearance();
                    }
                    setState(State.IN);
                }
                if (getState() == State.DUMPING && getSecondsSpentInState() > dumpTimeOut + getState().timeOut ||
                getState() == State.SOFT_DUMP && getSecondsSpentInState() > softDumpTimeOut + getState().timeOut) {
                    if (opModeType == OpModeType.AUTO || farDeposit() || closeDeposit() || crossDeposit()) allowLift = false;
                    if (intake.getState() == Intake.State.IN) {
                        intake.createClearance();
                    }
                    setState(State.IN);
                    isLoaded = false;
                    freight = Freight.NONE;
                }
                break;
            case RESETTING_ENCODER:
                allowTransfer = false;
                holdingPosition();
                slides.setPower(-1);
                if ((getSecondsSpentInState() > 0.3 && slides.getCurrent(CurrentUnit.AMPS) > liftCurrentLimit) || getSecondsSpentInState() >= 1.5) {
                    slides.setPower(0);
                    slides.getMotors()[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slides.getMotors()[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    setState(State.IN);
                }
                break;
        }
        double power = pidController.update(ticksToInches(slides.getCurrentPosition()));
        if (getState() != State.RESETTING_ENCODER) {
            slides.setPower(power);
        }
        if (intake.getState() == Intake.State.OUT) {
            Deposit.isLoaded = false;
            setState(State.IN);
        }
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
            Context.packet.put("Lift Current", slides.getCurrent(CurrentUnit.AMPS));
            Context.packet.put("Lift Motor Power", power);
            Context.packet.put("Lift Error", getLastError());
            Context.packet.put("isLoaded", isLoaded);
            Context.packet.put("Arm Real Position", arm.getServos().getRealPosition());
            Context.packet.put("Extension Real Displacement", extension.getServos().getRealPosition());
            Context.packet.put("IsFarDeposit", farDeposit());
        }
        //Context.packet.put("Freight", freight);
        // Context.packet.put("Level", defaultLevel);
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

    public void toggleFarDeposit() {
        if (distance != Distance.FAR) distance = Distance.FAR;
        else distance = Distance.CLOSE;
        intake.setDontFlipOut(farDeposit());
    }

    public void toggleCloseDeposit() {
        distance = Distance.CLOSE;
        intake.setDontFlipOut(false);
    }

    public void toggleCrossDeposit() {
        if (distance != Distance.CROSS) distance = Distance.CROSS;
        else distance = Distance.CLOSE;
        if (opModeType == OpModeType.TELE) allowLift = distance != Distance.CROSS;
        intake.setDontFlipOut(crossDeposit());
    }

    public void toggleMediumDeposit() {
        if (distance != Distance.MEDIUM) distance = Distance.MEDIUM;
        else distance = Distance.CLOSE;
        if (opModeType == OpModeType.TELE) allowLift = distance == Distance.MEDIUM;
        intake.setDontFlipOut(false);
    }

    public void setShouldCounterBalance(boolean counterBalance) {
        this.shouldCounterBalance = counterBalance;
    }

    public void toggleShouldCounterBalance() {
        shouldCounterBalance = !shouldCounterBalance;
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
        switch (getLevel()) {
            case LEVEL3:
                if (closeDeposit() || crossDeposit()) {
                    outPosition3 += pwr * outOffsetIncrement;
                } else {
                    outPosition3WithLift += pwr * outOffsetIncrement;
                }
                break;
            case LEVEL2:
                outPosition2 += pwr * outOffsetIncrement;
                break;
            case LEVEL1:
                outPosition1 += pwr * outOffsetIncrement;
                break;
            case SHARED:
                if (crossDeposit()) {
                    outPositionSharedFar += pwr * outOffsetIncrement;
                } else {
                    outPositionShared += pwr * outOffsetIncrement;
                }
                break;
        }
        // offsetOutPosition = offsetOutPosition + pwr * outOffsetIncrement;
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
        if (distance == Distance.FAR) return Level.LEVEL3;
        if (freight == Freight.BALL && opModeType == OpModeType.TELE && defaultLevel == Level.LEVEL3) return Level.LEVEL2;
        return defaultLevel;
    }

    public static double weightSlides = 0.9, weightArm = 0.5, weightExtension = 0.4;

    public Vector2d getModuleWeightedVector() {
        Vector2d slidesVec = new Vector2d(
                ticksToInches(slides.getCurrentPosition()) * Math.cos(angle),
                ticksToInches(slides.getCurrentPosition()) * Math.sin(angle)
        );
        Vector2d extensionVec = new Vector2d().plus(slidesVec);
        //extensionVec = new Vector2d(extension.getEstimatedDisplacement()).div(2).plus(slidesVec);
        //Vector2d armVec = arm.getVector().plus(extensionVec);
        return ((slidesVec.times(weightSlides))
                .plus((extensionVec).times(weightExtension))
                //.plus(armVec.times(weightArm)))
                .div(weightSlides + weightExtension));
    }

    public Vector2d getModuleWeightedVelocityVector() {
        Vector2d slidesVec = new Vector2d(
                ticksToInches(slides.getVelocity()) * Math.cos(angle),
                ticksToInches(slides.getVelocity()) * Math.sin(angle)
        );
        Vector2d extensionVec = new Vector2d().plus(slidesVec);
        if (platformIsOut() && extension.isTransitioning()) {
            extensionVec = new Vector2d(26.4252).div(2).plus(slidesVec);
        }
        //Vector2d armVec = arm.getVector().plus(extensionVec);
        return ((slidesVec.times(weightSlides))
                .plus((extensionVec).times(weightExtension))
                //.plus(armVec.times(weightArm)))
                .div(weightSlides + weightExtension));
    }

    public double getWeightedDisplacement() {
        Vector2d vec = getModuleWeightedVector();
        return Math.hypot(vec.getX(), vec.getY());
    }

    public double getWeightedVelocity() {
        Vector2d vec = getModuleWeightedVelocityVector();
        return Math.hypot(vec.getX(), vec.getY());
    }

    public double getWeight() {
        return weightSlides + weightExtension;
    }

    public static double LEVEL3 = 11.5;
    public static double LEVEL2 = 0;
    public static double LEVEL1 = 0;
    public static double allowableDepositError = 8.0;
    public static double angle = Math.toRadians(30);

    private double getLevelHeight(Level state) {
        if (closeDeposit()) return 0;
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
    private double outPosition(Level level) {
        double outPos = outPosition3;
        switch (level) {
            case LEVEL3:
                outPos = closeDeposit() || crossDeposit() ? outPosition3 : outPosition3WithLift;
                break;
            case LEVEL2:
                outPos = outPosition2;
                break;
            case LEVEL1:
                outPos = outPosition1;
                break;
            case SHARED:
                outPos = crossDeposit() ? outPositionSharedFar : outPositionShared;
                break;
        }
        if (closeDeposit()) outPos += 0.02;
        return outPos + offsetOutPosition;
    }

    private double extendPosition(Level level) {
        double extendPos = extendOut3;
        switch (level) {
            case LEVEL3:
                extendPos = closeDeposit() ? 0.0 : extendIn;
                break;
            case LEVEL2:
                extendPos = extendOut2;
                break;
            case LEVEL1:
                extendPos = extendOut1;
                break;
            case SHARED:
                extendPos = crossDeposit() ? extendOutSharedFar : extendOutShared;
                break;
        }
        return farDeposit() ? 0.0 : extendPos + offsetExtendPosition;
    }

    /**
     * Extends the platform out
     */
    private void flipOut(Level state) {
        double position = outPosition(state);
        if (toggle) position -= 0.001;
        double extensionPos = extendPosition(state);
        extension.setPosition(extensionPos);
        // speed * time
        // arm.setPosition(lastOutPosition + (position - lastOutPosition) * getSecondsSpentInState() / flipOutTime);
        if (getLevel() == Level.LEVEL1) {
            if (extension.getServos().getError() < 0.15) {
                arm.setPosition(position);
            }
        } else {
            arm.setPosition(position);
        }
    }

    private void holdingPosition() {
        double position = holdingPosition;
        arm.setPosition(position);
        extension.setPosition(extendIn);
    }

    /**
     * Return platform to rest
     */
    private void flipIn() {
        double position = inPosition;
        extension.setPosition(extendIn);
        if (!extension.isTransitioning() && Math.abs(ticksToInches(slides.getCurrentPosition())) < 6.0) {
            if ((intake.getState() == Intake.State.IN && intake.getContainsBlock()) || intake.getState() == Intake.State.TRANSFER) arm.setPosition(position);
            else arm.setPosition(higherInPosition);
        } else {
            arm.setPosition(0.5);
        }
    }

    /**
     * Dumps the loaded element onto hub
     */
    public void dump() {
        if (getState() == State.OUT)
            setState(State.DUMPING);
    }

    /**
     * Softly dumps the loaded element onto hub
     */
    public void softDump() {
        if (getState() == State.OUT)
            setState(State.SOFT_DUMP);
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
        return (isDumping() && getSecondsSpentInState() < (getState().timeOut + dumpTimeOut) * 0.4) || platformIsOut();
    }

    public boolean platformIsOut() {
        return getState() == State.OUT;
    }

    public boolean isDumping() {
        return getState() == State.DUMPING || getState() == State.SOFT_DUMP;
    }

    public void resetEncoder() {
        setState(State.RESETTING_ENCODER);
    }
}
