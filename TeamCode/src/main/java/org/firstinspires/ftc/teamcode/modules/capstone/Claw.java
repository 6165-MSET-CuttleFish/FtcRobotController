package org.firstinspires.ftc.teamcode.modules.capstone;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

/**
 * Module to collect the team marker at the start of the match
 * @author Sreyash Das Sarma
 */
@Config
public class Claw extends Module<Claw.State> {
    @Override
    public boolean isTransitioningState() {
        return false;
    }

    public enum State implements StateBuilder {
        TRANSIT_IN (0.5),
        IN(1),
        TRANSIT_OUT(1),
        OUT(0.4),
        HALF(0.2);
        final double time;
        State(double time) {
            this.time = time;
        }

        @Override
        public Double getTimeOut() {
            return time;
        }
    }

    Servo slide;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Claw(HardwareMap hardwareMap) {
        super(hardwareMap, State.IN);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        slide = hardwareMap.servo.get("capstoneLowerLift");
    }

    /**w
     * This function updates all necessary controls in a loop
     */
    @Override
    public void internalUpdate() {
        switch (getState()) {
            case TRANSIT_IN:
                if (getSecondsSpentInState() > getState().time) {
                    setState(Claw.State.IN);
                }
            case IN:
                in();
                break;
            case TRANSIT_OUT:
                if (getSecondsSpentInState() > getState().time) {
                    setState(Claw.State.OUT);
                }
            case OUT:
                out();
                break;
            case HALF:
                halfCase();
                break;

        }
    }

    private void out() {
        slide.setPosition(0);
    }

    private void halfCase() {
        slide.setPosition(0.05);
    }

    public static double inPos = 0.25;

    private void in() {
        slide.setPosition(inPos);
    }

    public void pickUp() {
        if (getState() != State.OUT) setState(State.TRANSIT_OUT);
    }

    public void dropDown() {
        if (getState() != State.IN) setState(State.TRANSIT_IN);
    }

    public void half() {
        if (getState() != State.HALF) setState(State.HALF);
    }

    @Override
    public boolean isDoingInternalWork() {
        return getState() == State.TRANSIT_OUT || getState() == State.TRANSIT_IN;
    }
}
