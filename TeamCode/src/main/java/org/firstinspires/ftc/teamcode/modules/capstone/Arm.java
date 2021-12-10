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
        TRANSIT_IN (0,1),
        IN(0.2,0.5),
        TRANSIT_OUT(0.5, 0.75),
        OUT(0.5,0.1),
        PRECAP(0,0.5),
        CAPOFFSET(0,0),
        CAP(0,0.3),
        IDLE(0,0);
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
                    setState(State.IN);
                }
            case IN:
                in();
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                out();
                break;
            case PRECAP:
                precappos();
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.CAPOFFSET);
                }
            case CAPOFFSET:
                break;
            case CAP:
                cappos();
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.IDLE);
                }
                break;

        }
    }

    private void out() {
        arm.setPosition(0.18); // 0.175
    }
    /**
     * Return platform to rest
     */
    private void in() {
         arm.setPosition(1);
    }
    private void precappos() {
        arm.setPosition(0.58);
    }
    private void cappos() {
        arm.setPosition(0.46);
    }

    public void ready(){
        if (getState() != State.OUT) setState(State.TRANSIT_OUT);
    }
    public void hold(){
        if (getState() != State.IN) setState(State.TRANSIT_IN);
    }
    public void precap(){
        setState(State.PRECAP);
    }
    public void cap(){
        setState(State.CAP);
    }
    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isDoingWork() {
        return getState() == State.TRANSIT_IN || getState() == Arm.State.TRANSIT_OUT;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

}
