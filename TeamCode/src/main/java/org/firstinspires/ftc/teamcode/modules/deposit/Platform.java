package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Srey Das Sarma
 */
public class Platform extends Module<Platform.State> {
    public enum State {
        TRANSIT_IN (0,0.5),
        IN(0.2,0.5),
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

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Platform(HardwareMap hardwareMap) {
        super(hardwareMap, State.IN);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        dump = hardwareMap.servo.get("depositDump");
        latch = hardwareMap.servo.get("depositLatch");
        setState(State.IN);
    }

    /**
     *
     */
    public void retrieve() {
        setState(State.IN);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case TRANSIT_IN:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.IN);
                }
            case IN:
                in();
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.OUT);
                }
                out();
                break;
            case OUT:
                out();
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.TRANSIT_IN);
                }
                break;
        }
    }

    /**
     * Extends the platform out
     */

    private void out() {
        dump.setPosition(0.6);
        latch.setPosition(0.75);
    }

    /**
     * Return platform to rest
     */
    private void in() {
        dump.setPosition(0.18);
        latch.setPosition(1);
    }
    /**
     * Dumps the loaded element onto hub
     */
    public void dump(){
        setState(Platform.State.TRANSIT_OUT);
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
