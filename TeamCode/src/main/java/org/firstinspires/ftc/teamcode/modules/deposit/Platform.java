package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Module;

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
    StateMachine<State> stateMachine;
    Servo platformL, platformR;
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
        platformL = hardwareMap.servo.get("platformL");
        platformR = hardwareMap.servo.get("platformR");
        setState(State.IN);
    }

    /**
     * Dumps the loaded element onto hub
     */
    public void dump() {
        setState(State.TRANSIT_OUT);
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
        platformL.setPosition(0.5);
        platformR.setPosition(0.5);
    }

    /**
     * Return platform to rest
     */
    private void in() {
        platformL.setPosition(0.1);
        platformR.setPosition(0.9);
    }
  
    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        /* Conditions:
         * elapsed time passes set time before module reaches position
         */
        if(platformL.getPosition()!= getState().angle&&elapsedTime.time()>getState().time){
            return true;
        } else return platformL.getPosition() != platformR.getPosition();
    }
  
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingWork() {
        return getState() == State.OUT || getState() == State.TRANSIT_OUT;
    }
}
