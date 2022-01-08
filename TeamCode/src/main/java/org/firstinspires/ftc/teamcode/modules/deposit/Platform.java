package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Balance;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

import static org.firstinspires.ftc.teamcode.util.field.Context.balance;
import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Martin
 */
@Config
public class Platform extends Module<Platform.State> {
    public static double outPosition3 = 0.8;
    public static double outPosition2 = 0.8;
    public static double outPosition1 = 0.8;
    public static double outPositionFar = 0.8;
    public static double tipDiff = 0.015;
    public static double inPosition = 0.1;
    public static double lockPosition = 0.49;
    public static double unlockPosition = 0.7;
    public static double sum = 1;
    public static double timeDiffBalance = 0.5;
    public static boolean isLoaded;
    public enum State implements StateBuilder {
        TRANSIT_IN (0.4),
        IN(0.5),
        HOLDING(0.0),
        CREATE_CLEARANCE(0.3),
        TRANSIT_OUT(0.3),
        OUT(0),
        DUMPING(0.5);
        final double time;
        @Override
        public double getTime() {
            double time = this.time;
            if (balance == Balance.AWAY) time += timeDiffBalance;
            return time;
        }
        State(double time) {
            this.time = time;
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
        super(hardwareMap, State.IN);
        this.intake = intake;
        this.deposit = deposit;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        dumpLeft = hardwareMap.servo.get("depositDumpL");
        dumpRight = hardwareMap.servo.get("depositDumpR");
        tilt = hardwareMap.servo.get("depositLatch");
        lock = hardwareMap.servo.get("lock");
        flipIn();
        tiltIn();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
        switch (getState()) {
            case TRANSIT_IN:
                isLoaded = false;
                if (getTimeSpentInState() > getState().time) {
                    setState(State.IN);
                }
            case IN:
                tiltOut();
                unlock();
                flipIn();
                Deposit.allowLift = false;
                if(!intake.isDoingWork() && isLoaded)
                    setState(State.CREATE_CLEARANCE);
                break;
            case CREATE_CLEARANCE:
                if (getTimeSpentInState() > getState().time) {
                    lock();
                    setState(State.HOLDING);
                }
            case HOLDING:
                holdingPosition();
                break;
            case TRANSIT_OUT:
                if (getTimeSpentInState() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                lock();
                flipOut();
                break;
            case DUMPING:
                unlock();
                flipOut();
                tiltIn();
                if (isLoaded ? getTimeSpentInState() > getState().getTime(): getTimeSpentInState() > getState().getTime()+ 0.8) {
                    setState(State.TRANSIT_IN);
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
        double position = (Deposit.allowLift || opModeType != OpModeType.TELE) ? outPosition() : 0.3;
        dumpLeft.setPosition(position);
        dumpRight.setPosition(sum - position);
    }

    private void holdingPosition() {
        double position = 0.3;
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
        setState(State.DUMPING);
    }

    /**
     * Dumps the loaded element onto hub
     */
    public void prepPlatform() {
        setState(State.TRANSIT_OUT);
    }

    private void tiltIn() {
        tilt.setPosition(0.95);
    }

    private void tiltOut() {
        tilt.setPosition(0.2);
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
        return getState() == State.DUMPING || getState() == State.OUT || getState() == State.TRANSIT_IN || getState() == State.HOLDING;
    }
}
