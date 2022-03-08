package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static double outPosition3 = 0.42;
    public static double outPosition2 = 0.13;
    public static double outPosition1 = 0.0;
    public static double holdingPosition = 0.7;
    public static double tipDiff = 0.015;
    public static double inPosition = 1, higherInPosition = 0.87;
    public static double lockPosition = 0.54;
    public static double unlockPosition = 0.29;
    public static double blockDistanceTolerance = 7;
    public static double dumpServoPositionPerSecond = 1.7;
    public static double flipServoPositionPerSecond = 2;
    public static boolean isLoaded;
    public static double tiltInPos = 0.79, tiltOutPos = 0, furtherInPosition = 0.9, tiltOutPos2 = 0.1, tiltOutPos1 = 0.1;

    @Override
    public boolean isTransitioningState() {
        return tilt.isTransitioning() || arm.isTransitioning();
    }

    public enum State implements StateBuilder {
        IN(0.5),
        CREATE_CLEARANCE,
        LOCKING(0.16),
        DUMPING(0.3),
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
    private ControllableServos arm, tilt, lock;
    private final Intake intake;
    private final Deposit deposit;
    private ColorRangeSensor blockDetector;
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
                dumpLeft = hardwareMap.servo.get("depositDumpL"),
                dumpRight = hardwareMap.servo.get("depositDumpR");
        dumpRight.setDirection(Servo.Direction.REVERSE);
        arm = new ControllableServos(dumpLeft, dumpRight);
        tilt = new ControllableServos(hardwareMap.servo.get("platformTilt"));
        lock = new ControllableServos(hardwareMap.servo.get("lock"));
        blockDetector = hardwareMap.get(ColorRangeSensor.class, "platformBlock");
        flipIn();
        tiltIn();
        unlock();
        if (opModeType == OpModeType.AUTO) lock();
        setActuators(arm, tilt, lock);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        arm.setPositionPerSecond(dumpServoPositionPerSecond);
        tilt.setPositionPerSecond(flipServoPositionPerSecond);
        double distance = blockDetector.getDistance(DistanceUnit.CM);
        if (intake.getState() == Intake.State.TRANSFER && distance < blockDistanceTolerance) {
            isLoaded = true;
        }
        switch (getState()) {
            case IN:
                unlock();
                tiltIn();
                flipIn();
                if (isLoaded) {
                    setState(State.LOCKING);
                }
                if (!intakeCleared && !arm.isTransitioning()) {
                    if (intake.getState() != Intake.State.OUT) intake.retractIntake();
                    intakeCleared = true;
                }
                if (intake.getState() == Intake.State.OUT && intake.isTransitioningState()) {
                    setState(State.CREATE_CLEARANCE);
                }
                break;
            case CREATE_CLEARANCE:
                arm.setPosition(higherInPosition);
                tilt.setPosition(furtherInPosition);
                if (intake.getState() != Intake.State.OUT) {
                    setState(State.IN);
                }
                break;
            case LOCKING:
                lock();
                if (getSecondsSpentInState() > getState().timeOut && (opModeType != OpModeType.AUTO || Deposit.allowLift)) {
                    prepPlatform(deposit.getDefaultState());
                }
                break;
            case OUT1:
            case OUT2:
            case OUT3:
                if (getState() == State.OUT1) {
                    intake.retractIntake();
                } else {
                    intake.counterBalance();
                }
                intakeCleared = false;
                setState(getNeededOutState(deposit.getDefaultState()));
                lock();
                flipOut(deposit.getDefaultState());
                tiltOut();
                break;
            case DUMPING:
                unlock();
                if (getSecondsSpentInState() > getState().timeOut) {
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
        arm.setPosition(position);
    }

    /**
     * Dumps the loaded element onto hub
     */
    public void dump() {
        if (getState() == State.OUT3 || getState() == State.OUT2 || getState() == State.OUT1)
            setState(State.DUMPING);
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

    private void tiltIn() {
        tilt.setPosition(tiltInPos);
    }

    private void tiltOut() {
        switch (deposit.getDefaultState()) {
            case LEVEL3:
                tilt.setPosition(tiltOutPos);
                break;
            case LEVEL2:
                tilt.setPosition(tiltOutPos2);
                break;
            case LEVEL1:
                tilt.setPosition(tiltOutPos1);
        }
        //tilt.setPosition(tiltOutPos);
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
