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
    public static double outPosition = 0.8;
    public static double tipDiff = 0.015;
    public static double inPosition = 0.1;
    public static double lockPosition = 0.49;
    public static double unlockPosition = 0.7;
    public static double sum = 1.03;
    public static double timeDiffBalance = 0.5;
    public static boolean isLoaded;
    public enum State implements StateBuilder {
        TRANSIT_IN (0.4),
        IDLE(0.5),
        TRANSIT_OUT(0.08),
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
    Servo dumpLeft, dumpRight, latch, lock;
    private final Intake intake;


    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Platform(HardwareMap hardwareMap, Intake intake) {
        super(hardwareMap, State.IDLE);
        this.intake = intake;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        dumpLeft = hardwareMap.servo.get("depositDumpL");
        dumpRight = hardwareMap.servo.get("depositDumpR");
        latch = hardwareMap.servo.get("depositLatch");
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
                    setState(State.IDLE);
                }
            case IDLE:
                tiltOut();
                unlock();
                flipIn();
                Deposit.allowLift = false;
                if(!intake.isDoingInternalWork() && isLoaded)
                    setState(State.TRANSIT_OUT);
                break;
            case TRANSIT_OUT:
                if (getTimeSpentInState() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                if (getState() == State.OUT) lock();
                flipOut();
                break;
            case DUMPING:
                lock();
                flipOut();
                tiltIn();
                if (isLoaded ? getTimeSpentInState() > getState().getTime(): getTimeSpentInState() > getState().getTime()+ 0.8) {
                    setState(State.TRANSIT_IN);
                }
                break;
        }
        if (intake.isDoingWork())
            setState(State.IDLE);
    }

    /**
     * @return servo position based on balance of hub
     */
    private double outPosition() {
        switch (balance) {
            case BALANCED:
            case TOWARD:
                return outPosition;
            case AWAY:
                return outPosition - tipDiff;
        }
        return outPosition;
    }

    /**
     * Extends the platform out
     */
    private void flipOut() {
        double position = (Deposit.allowLift || opModeType != OpModeType.TELE) ? outPosition() : 0.5;
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

    private void tiltIn() {
        latch.setPosition(0.95);
        // tilt platform in
    }

    private void tiltOut() {
        latch.setPosition(0.2);
        // tilt platform out
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
        return getState() == State.DUMPING || getState() == State.OUT || getState() == State.TRANSIT_IN;
    }
}
