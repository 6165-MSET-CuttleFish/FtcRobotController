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
    public static double tipDiff = 0.015;
    public static double inPosition = 0.82;
    public static double lockPosition = 0.49;
    public static double unlockPosition = 0.7;
    public static double sum = 1;
    public static double timeDiffBalance = 0.5;
    public static boolean isLoaded;
    public enum State implements StateBuilder {
        TRANSIT_IN (0.7, 0),
        IN(0.5, 0),
        HOLDING(0.0, 0.5),
        TRANSIT_OUT(0.3, 1),
        OUT(0, 1),
        DUMPING(0.5),
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
    Servo dumpLeft, dumpRight, tilt, lock;
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

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        switch (getState()) {
            case TRANSIT_IN:
                isLoaded = false;
                if (getTimeSpentInState() > getState().timeOut) {
                    setState(State.IN);
                }
            case IN:
                tiltIn();
                unlock();
                flipIn();
                if (isLoaded && intake.getState() == Intake.State.CREATE_CLEARANCE) {
                    prepPlatform();
                }
                break;
            case HOLDING:
                //setState(State.TRANSIT_OUT);
                break;
            case TRANSIT_OUT:
                if (hasExceededTimeOut()) {
                    setState(State.OUT);
                }
            case OUT:
                lock();
                flipOut();
                tiltOut();
                break;
            case DUMPING:
                unlock();
                flipOut();
                if (isLoaded ? getTimeSpentInState() > getState().getTimeOut(): getTimeSpentInState() > getState().getTimeOut()+ 0.8) {
                    setState(State.TRANSIT_IN);
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
    private double outPosition() {
        double outPos = outPosition3;
        switch (deposit.getState()) {
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
    private void flipOut() {
        double position = outPosition();
        dumpLeft.setPosition(position);
        dumpRight.setPosition(sum - position);
    }

    private void holdingPosition() {
        double position = 0.4;
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
        if (getState() == State.OUT || getState() == State.TRANSIT_OUT) setState(State.DUMPING);
    }

    /**
     * Puts platform into the prepped position
     */
    public void prepPlatform() {
        if (getState() != State.OUT) setState(State.TRANSIT_OUT);
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
        return getState() == State.DUMPING || getState() == State.OUT || getState() == State.TRANSIT_IN || getState() == State.HOLDING || getState() == State.TRANSIT_OUT;
    }
}
