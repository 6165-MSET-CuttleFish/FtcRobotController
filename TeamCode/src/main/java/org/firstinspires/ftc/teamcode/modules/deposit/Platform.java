package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.Details;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Srey Das Sarma
 */
public class Platform extends Module<Platform.State> {
    public enum State {
        TRANSIT_IN (0,0.5),
        IDLE(0.2,0.5),
        TRANSIT_OUT(0.5, 0.5),
        OUT(0.5,0.1);
        final double angle;
        final double time;
        State(double angle,double time) {
            this.angle = angle;
            this.time = time;
        }
    }
    Servo dump, latch;
    private Intake intake;
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
        dump = hardwareMap.servo.get("depositDump");
        latch = hardwareMap.servo.get("depositLatch");
        setState(State.IDLE);
    }

    /**
     *
     */
    public void retrieve() {
        setState(State.IDLE);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case TRANSIT_IN:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.IDLE);
                }
            case IDLE:
                closeLatch();
                in();
                if(intake.getState() == Intake.State.IN && isLoaded)
                    setState(State.TRANSIT_OUT);
                break;
            case TRANSIT_OUT:
                if(isLoaded && getState().time > elapsedTime.seconds())
                    setState(State.OUT);
                out();
                break;
            case OUT:
                out();
                openLatch();
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.TRANSIT_IN);
                }
                break;
        }
        Details.telemetry.addData("State", getState());
        Details.telemetry.update();
    }

    /**
     * Extends the platform out
     */

    private void out() {
        dump.setPosition(0.4);
    }

    /**
     * Return platform to rest
     */
    private void in() {
        dump.setPosition(0.82);
    }
    /**
     * Dumps the loaded element onto hub
     */
    public void dump(){
        setState(State.OUT);
    }
    private void openLatch() {
        latch.setPosition(0.75);
        isLoaded = false;
    }
    private void closeLatch() {
        latch.setPosition(1);
    }
    /**
     * @return Whether the elapsed time passes set time before module reaches position
     */
    @Override
    public boolean isHazardous() {
        if(dump.getPosition()!= getState().angle && elapsedTime.time()>getState().time){
            return true;
        }
        return false;
    }
  
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingWork() {
        return getState() == State.OUT || getState() == State.TRANSIT_OUT;
    }
}
