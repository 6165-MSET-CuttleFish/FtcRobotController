package org.firstinspires.ftc.teamcode.modules.capstone;

import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Module to collect the team marker at the start of the match
 * @author Sreyash Das Sarma
 */
public class Arm extends Module<Arm.State> {
    public enum State {
        TRANSIT_IN (0,0.2),
        IN(0.2,0.5),
        TRANSIT_OUT(0.5, 0.1),
        OUT(0.5,0.1);
        final double dist;
        final double time;
        State(double dist,double time) {
            this.dist = dist;
            this.time = time;
        }
    }
    Servo arm;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Arm(HardwareMap hardwareMap) {
        super(hardwareMap, State.IN);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        arm = hardwareMap.servo.get("capstoneArm");
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case TRANSIT_IN:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Arm.State.IN);
                }
            case IN:
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Arm.State.OUT);
                }
                break;
            case OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.TRANSIT_IN);
                }
                break;

        }
    }

    public void out() {
        arm.setPosition(0.199);
    }
    /**
     * Return platform to rest
     */
    private void in() {
        arm.setPosition(0.8);
    }

    public void ready(){
        setState(Arm.State.TRANSIT_OUT);
        out();
    }
    public void hold(){
        setState(Arm.State.TRANSIT_IN);
        in();
    }
    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isDoingWork() {
        return getState() == Arm.State.OUT || getState() == Arm.State.TRANSIT_OUT;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

}
