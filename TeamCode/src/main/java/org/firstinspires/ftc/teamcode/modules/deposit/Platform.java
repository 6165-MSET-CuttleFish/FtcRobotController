package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

import static org.firstinspires.ftc.teamcode.util.field.Context.balance;
import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Ayush Raman
 */
@Config
public class Platform extends Module<Platform.State> {
    public static double outPosition3 = 0.47;
    public static double outPosition2 = 0.41;
    public static double outPosition1 = 0.41;
    public static double extendIn = 0.26, extendOut = 0.0;
    public static double holdingPosition = 0.7;
    public static double tipDiff = 0.004;
    public static double inPosition = 0.663, higherInPosition = 0.64;
    public static double lockPosition = 0.67;
    public static double unlockPosition = 0.57;
    public static double kickPosition = 0.95;
    public static double blockDistanceTolerance = 9;
    public static double dumpServoPositionPerSecond = 0.5;
    public static double extensionServoPositionPerSecond = 0.6;
    public static boolean isLoaded;
    public static boolean shouldCounterBalance = true;
    public static double dumpTimeOut = 0.13;
    @Override
    public boolean isTransitioningState() {
        return extension.isTransitioning() || arm.isTransitioning();
    }

    public enum State implements StateBuilder {
        IN(0.5),
        CREATE_CLEARANCE,
        LOCKING(0.16),
        DUMPING(0.2),
        SOFT_DUMP,
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
    private ControllableServos arm, lock, extension;
    private final Intake intake;
    private final Deposit deposit;
    boolean intakeCleared;


    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Platform(HardwareMap hardwareMap, Intake intake, Deposit deposit) {
        super(hardwareMap, opModeType == OpModeType.AUTO ? State.LOCKING : State.IN, new Pose2d());
        this.intake = intake;
        this.deposit = deposit;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        Servo
                dumpLeft = hardwareMap.servo.get("depositDumpL");
                //dumpRight = hardwareMap.servo.get("depositDumpR");
        arm = new ControllableServos(dumpLeft);
        Servo
                extL = hardwareMap.servo.get("extL"),
                extR = hardwareMap.servo.get("extR");
        extR.setDirection(Servo.Direction.REVERSE);
        extension = new ControllableServos(extL, extR);
        lock = new ControllableServos(hardwareMap.servo.get("lock"));
        flipIn();
        flipIn();
        unlock();
        if (opModeType == OpModeType.AUTO) lock();
        setActuators(arm, extension, lock);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        arm.setPositionPerSecond(dumpServoPositionPerSecond);
        extension.setPositionPerSecond(extensionServoPositionPerSecond);
        double distance;
        try {
            distance = blockDistanceTolerance + 1;
        } catch (Exception e) {
            distance = 1;
        }
        if (intake.getState() == Intake.State.TRANSFER && distance < blockDistanceTolerance) {
            isLoaded = true;
        }
        switch (getState()) {
            case IN:
                if (!Platform.isLoaded) unlock();
                flipIn();
                if (isLoaded) {
                    setState(State.LOCKING);
                }
                if (!intakeCleared && !isTransitioningState()) {
                    if (intake.getState() != Intake.State.OUT) intake.retractIntake();
                    intakeCleared = true;
                }
                if (intake.getState() == Intake.State.OUT && intake.isTransitioningState()) {
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
                if (getSecondsSpentInState() > getState().timeOut && Deposit.allowLift) {
                    prepPlatform(deposit.getDefaultState());
                }
                break;
            case OUT1:
            case OUT2:
            case OUT3:
                if (getState() == State.OUT1 || !shouldCounterBalance) {
                    intake.retractIntake();
                } else {
                    intake.counterBalance();
                }
                intakeCleared = false;
                setState(getNeededOutState(deposit.getDefaultState()));
                if (!Deposit.allowLift) {
                    flipIn();
                    setState(State.IN);
                }
                lock();
                flipOut(deposit.getDefaultState());
                break;
            case DUMPING:
            case SOFT_DUMP:
                double diff = kickPosition - lockPosition;
                if (getState() == State.DUMPING) lock.setPosition(Range.clip(lockPosition + diff * getSecondsSpentInState() / dumpTimeOut, lockPosition, kickPosition));
                else
                if (!Deposit.allowLift) {
                    if (getPreviousState() == State.OUT1) {
                        intake.createClearance();
                    }
                    setState(State.IN);
                }
                if (getSecondsSpentInState() > dumpTimeOut) {
                    Deposit.allowLift = false;
                    if (getPreviousState() == State.OUT1) {
                        intake.createClearance();
                    }
                    setState(State.IN);
                    isLoaded = false;
                }
                break;
        }
        if (isDebugMode()) {
            Context.packet.put("isLoaded", isLoaded);
            Context.packet.put("Arm Real Position", arm.getRealPosition());
            Context.packet.put("Platform DS Distance", distance);
            Context.packet.put("Extension Real Position", extension.getRealPosition());
        }
    }

    /**
     * @return servo position based on balance of hub
     */
    private double outPosition(Deposit.State state) {
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
        if (state != Deposit.State.LEVEL3) {
            switch (balance) {
                case BALANCED:
                case TOWARD:
                    return outPos - tipDiff;
                case AWAY:
                    return outPos + tipDiff;
            }
        }
        return outPos;
    }

    /**
     * Extends the platform out
     */
    private void flipOut(Deposit.State state) {
        double position = outPosition(state);
        arm.setPosition(position);
        extension.setPosition(extendOut);
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
        if (!extension.isTransitioning()) arm.setPosition(position);
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

    private State getNeededOutState(Deposit.State state) {
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
    public void prepPlatform(Deposit.State state) {
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
        return getState() == State.DUMPING || platformIsOut(getState());
    }

    public boolean platformIsOut(State state) {
        return state == State.OUT1 || state == State.OUT2 || state == State.OUT3;
    }
}
