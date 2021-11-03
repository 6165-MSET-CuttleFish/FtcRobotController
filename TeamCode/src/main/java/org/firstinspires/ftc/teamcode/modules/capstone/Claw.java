package org.firstinspires.ftc.teamcode.modules.capstone;

import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.deposit.Platform;

/**
 * Module to collect the team marker at the start of the match
 * @author Sreyash Das Sarma
 */
public class Claw extends Module<Claw.State> {
    public enum State {
        TRANSIT_IN (0,0.5),
        IN(0.2,0.5),
        IDLEOut(0,0.5),
        TRANSIT_OUT(0.5, 0.5),
        OUT(0.5,0.1);
        final double dist;
        final double time;
        State(double dist,double time) {
            this.dist = dist;
            this.time = time;
        }
    }
    StateMachine<State> stateMachine;
    Servo clawL, clawR;
    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Claw(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLEOut);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
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
                    setState(Claw.State.IN);
                }
            case IN:
                close();
                break;
            case IDLEOut:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.TRANSIT_OUT);
                }
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Claw.State.OUT);
                }
                open();
                break;
            case OUT:
                open();
                if (elapsedTime.seconds() > getState().time) {
                    setState(Claw.State.TRANSIT_IN);
                }
                break;
        }
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    public void open() {
        clawL.setPosition(0.0);
        clawR.setPosition(0.26);
    }

    /**
     * Return platform to rest
     */
    public void close() {
        clawL.setPosition(0.96);
        clawR.setPosition(0.7);
    }
    @Override
    public boolean isDoingWork() {
        if(clawL.getPosition()!= getState().dist&&elapsedTime.time()>getState().time){
            return true;
        } else return clawL.getPosition() != clawR.getPosition();
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isHazardous() {
        return getState() == Claw.State.OUT || getState() == Claw.State.TRANSIT_OUT;
    }

}
