package org.firstinspires.ftc.teamcode.modules.capstone;

import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Module to collect the team marker at the start of the match
 * @author Sreyash Das Sarma
 */
public class Slides extends Module<Slides.State> {
    public enum State {
        TRANSIT_IN (0,1),
        IN(0.2,1),
        TRANSIT_OUT(0.5, 0.5),
        OUT(0.5,0.4),
        HALF(0,0.5);
        final double dist;
        final double time;
        State(double dist,double time) {
            this.dist = dist;
            this.time = time;
        }
    }
    Servo slideLeft;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Slides(HardwareMap hardwareMap) {
        super(hardwareMap, State.IN);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        slideLeft = hardwareMap.servo.get("capstoneLowerLift");
    }

    /**w
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case TRANSIT_IN:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Slides.State.IN);
                }
            case IN:
                in();
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Slides.State.OUT);
                }
            case OUT:
                out();
                break;
            case HALF:
                halfcase();
                break;

        }
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    private void out() {
        placeset(0);
    }

    private void halfcase() {
        placeset(0.05);
    }
    /**
     * Return platform to rest
     */
    private void in() {
        placeset(0.2);
    }

    public void pickUp() {
        if (getState() != State.TRANSIT_OUT) setState(State.TRANSIT_OUT);
    }

    public void dropDown() {
        if (getState() != State.TRANSIT_IN) setState(State.TRANSIT_IN);
    }

    public void half() {
        if (getState() != State.HALF) setState(State.HALF);
    }

    @Override
    public boolean isDoingWork() {
        return getState() == State.TRANSIT_OUT || getState() == State.TRANSIT_IN;
    }
    /**
     * @return Whether the module is currently in a hazardous state
     */
    private void placeset(double pos){
        slideLeft.setPosition(pos);
    }
    @Override
    public boolean isHazardous() {
        return getState() == Slides.State.OUT || getState() == Slides.State.TRANSIT_OUT;
    }

}
