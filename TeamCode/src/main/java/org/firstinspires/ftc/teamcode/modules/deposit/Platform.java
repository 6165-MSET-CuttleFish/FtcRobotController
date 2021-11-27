package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;

import static org.firstinspires.ftc.teamcode.util.field.Details.balance;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Martin
 */
public class Platform extends Module<Platform.State> {
    public enum State {
        TRANSIT_IN (0.4),
        IDLE(0.5),
        TRANSIT_OUT(0.08),
        OUT(0),
        DUMPING(0.3);
        final double time;
        State(double time) {
            this.time = time;
        }
    }
    Servo dumpLeft, dumpRight, latch, lock;
    private final Intake intake;
    public static boolean isLoaded;


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
        setState(State.IDLE);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case TRANSIT_IN:
                isLoaded = false;
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.IDLE);
                }
            case IDLE:
                closeLatch();
                unlock();
                in();
                if(intake.getState() == Intake.State.IN && isLoaded)
                    setState(State.TRANSIT_OUT);
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                if (getState() == State.OUT) lock();
                out();
                break;
            case DUMPING:
                lock();
                out();
                openLatch();
                if (isLoaded ? elapsedTime.seconds() > getState().time : elapsedTime.seconds() > getState().time + 0.8) {
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
                return 0.58;
            case AWAY: return 0.54;
        }
        return 0.58;
    }

    /**
     * Extends the platform out
     */
    private void out() {
        double position = outPosition();
        dumpLeft.setPosition(position);
        dumpRight.setPosition(1 - position);
    }

    /**
     * Return platform to rest
     */
    private void in() {
        double position = 0.18;
        dumpLeft.setPosition(position);
        dumpRight.setPosition(1 - position);
    }
    /**
     * Dumps the loaded element onto hub
     */
    public void dump() {
        setState(State.DUMPING);
    }

    private void openLatch() {
        latch.setPosition(0.75);
    }

    private void closeLatch() {
        latch.setPosition(1);
    }

    private void lock() {
        lock.setPosition(0);
    }

    private void unlock() {
        lock.setPosition(0.71);
    }

    /**
     * @return Whether the elapsed time passes set time before module reaches position
     */
    @Override
    public boolean isHazardous() {
        return false;
    }
  
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingWork() {
        return getState() == State.DUMPING || getState() == State.OUT;
    }
}
