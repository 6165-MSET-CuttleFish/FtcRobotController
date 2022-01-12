package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Balance;

import androidx.annotation.NonNull;
import kotlin.jvm.functions.Function0;

import static org.firstinspires.ftc.teamcode.util.field.Context.balance;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Martin
 */
@Config
public class Platform extends Module<Platform.State> {
    public static double outPosition3 = 0.1;
    public static double outPosition2 = 0.1;
    public static double outPosition1 = 0.1;
    public static double outPositionFar = 0.1;
    public static double holdingPosition = 0.4;
    public static double tipDiff = 0.015;
    public static double inPosition = 0.82;
    public static double lockPosition = 0.49;
    public static double unlockPosition = 0.7;
    public static double sum = 1;
    public static double timeDiffBalance = 0.5;
    public static boolean isLoaded;
    public enum State implements StateBuilder {
        IN(0.5, 0),
        HOLDING(0.1, 0.5),
        LOCKING(0.2, 0),
        DUMPING(0.5, 1),
        OUT1(0.0, 1),
        OUT2(0, 0.9),
        OUT3(0, 0.7);
        private final double timeOut;
        private final double percentMotion;
        @Override
        public double getTimeOut() {
            double time = this.timeOut;
            if (balance == Balance.AWAY) time += timeDiffBalance;
            return time;
        }
        State(double timeOut, double motionProfile) {
            this.timeOut = timeOut;
            this.percentMotion = motionProfile;
        }
        State(double timeOut) {
            this.timeOut = timeOut;
            percentMotion = 0;
        }

        @Override
        public double getPercentMotion() {
            return percentMotion;
        }
    }
    private Servo dumpLeft, dumpRight, tilt, lock;
    private final Intake intake;
    private final Deposit deposit;


    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Platform(HardwareMap hardwareMap, Intake intake, Deposit deposit) {
        super(hardwareMap, State.IN, new Pose2d(), 1);
        this.intake = intake;
        this.deposit = deposit;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        dumpLeft = hardwareMap.servo.get("depositDumpL");
        dumpRight = hardwareMap.servo.get("depositDumpR");
        tilt = hardwareMap.servo.get("platformTilt");
        lock = hardwareMap.servo.get("lock");
        flipIn();
        tiltIn();
        unlock();
    }

    boolean intakeCleared;
    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        switch (getState()) {
            case IN:
                tiltIn();
                if (!isLoaded) unlock();
                if (deposit.getLastError() < Deposit.allowableDepositError) flipIn();
                if (isLoaded) {
                    setState(State.LOCKING);
                }
            case LOCKING:
                if (isTransitioningState() && !intakeCleared) {
                    intake.createClearance();
                    intakeCleared = true;
                } else if (!isTransitioningState()){
                    intakeCleared = false;
                }
                if (isLoaded) lock();
                if (getTimeSpentInState() > getState().timeOut && getState() == State.LOCKING) {
                    prepPlatform(deposit.getDefaultState());
                }
                break;
            case HOLDING:
                intakeCleared = false;
                holdingPosition();
                if (getTimeSpentInState() > getState().timeOut && deposit.getLastError() < Deposit.allowableDepositError) {
                    if (isLoaded) setState(getNeededOutState(deposit.getDefaultState()));
                    else setState(State.IN);
                }
                break;
            case OUT1:
            case OUT2:
            case OUT3:
                setState(getNeededOutState(deposit.getDefaultState()));
                lock();
                flipOut(deposit.getDefaultState());
                tiltOut();
                break;
            case DUMPING:
                unlock();
                if (isLoaded ? getTimeSpentInState() > getState().getTimeOut(): getTimeSpentInState() > getState().getTimeOut()+ 0.8) {
                    setState(State.HOLDING);
                    isLoaded = false;
                }
                break;
        }
        if (intake.isDoingWork())
            setState(State.IN);
    }

    /**
     * @return servo position based on balance of hub
     */
    private double outPosition(Deposit.State state) {
        double outPos = outPosition3;
        switch (state) {
            case LEVEL3:
                outPos = Deposit.farDeposit ? outPositionFar : outPosition3;
                break;
            case LEVEL2:
                outPos = Deposit.farDeposit ? outPositionFar : outPosition2;
                break;
            case LEVEL1:
                outPos = Deposit.farDeposit ? outPositionFar : outPosition1;
                break;
        }
        switch (balance) {
            case BALANCED:
            case TOWARD:
                return outPos;
            case AWAY:
                return outPos - tipDiff;
        }
        return outPos;
    }

    /**
     * Extends the platform out
     */
    private void flipOut(Deposit.State state) {
        double position = outPosition(state);
        dumpLeft.setPosition(position);
        dumpRight.setPosition(sum - position);
    }

    private void holdingPosition() {
        double position = holdingPosition;
        dumpLeft.setPosition(position);
        dumpRight.setPosition(sum - position);
    }

    /**
     * Return platform to rest
     */
    private void flipIn() {
        double position = inPosition;
        dumpLeft.setPosition(position);
        dumpRight.setPosition(sum - position);
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

    /**
     * Safety position for platform
     */
    public void safetyPosition() {
        setState(State.HOLDING);
    }

    public static double tiltInPos = 0.8, tiltOutPos = 0;

    private void tiltIn() {
        tilt.setPosition(tiltInPos);
    }

    private void tiltOut() {
        tilt.setPosition(tiltOutPos);
    }

    private void lock() {
        lock.setPosition(lockPosition);
    }

    private void unlock() {
        lock.setPosition(unlockPosition);
    }

    /**
     * @return Whether the elapsed time passes set time before module reaches position
     */
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }
  
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingInternalWork() {
        return getState() == State.DUMPING || platformIsOut(getState()) || (getState() == State.HOLDING && isLoaded);
    }

    public boolean platformIsOut(State state) {
        return state == State.OUT1 || state == State.OUT2 || state == State.OUT3;
    }
}
