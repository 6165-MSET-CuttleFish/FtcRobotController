package org.firstinspires.ftc.teamcode.modules.weigher;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Weighing the Freight to determine type
 *
 * @author Matthew Song
 */
public class Weigher extends Module<Weigher.State> {
    enum State {
        NONE,
        BALL,
        CUBE_LIGHT,
        CUBE_MEDIUM,
        CUBE_HEAVY,
        DUCK
    }
    State state = State.NONE;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Weigher(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        weighing();
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        //insert servos and sensors and stuff
        // led light to reveal object?
        // probably gradient for cubes, yellow for duck, white for ball
    }

    /**
     * @return The state of the module
     */
    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean isDoingWork() {
        return false;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

    private void weighing(){
        if(/*no thing*/true){
            setState(State.NONE);
            //set led off
        }
        else if(true/*light cube*/){
            setState(State.CUBE_LIGHT);
            //set led light
        }
        else if(true/*medium cube*/){
            setState(State.CUBE_MEDIUM);
            //set led med
        }
        else if(true/*heavy cube*/){
            setState(State.CUBE_HEAVY);
            //set led dark
        }
        else if(true/*ball*/){
            setState(State.BALL);
            //set led white
        }
        else if(true/*duck*/){
            setState(State.DUCK);
            //set led yellow
        }
    }
}
