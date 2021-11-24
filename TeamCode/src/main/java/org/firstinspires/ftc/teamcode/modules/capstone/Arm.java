package org.firstinspires.ftc.teamcode.modules.capstone;

import com.arcrobotics.ftclib.hardware.ServoEx;
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
        TRANSIT_IN (0,0.5),
        IN(0.2,0.5),
        TRANSIT_OUT(0.5, 0.5),
        OUT(0.5,0.1),
        IDLE(0,0);
        final double dist;
        final double time;
        State(double dist,double time) {
            this.dist = dist;
            this.time = time;
        }
    }
    ServoEx arm;

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
        arm = hardwareMap.get(ServoEx.class,"arm");
        setState(State.IN);
    }

    /**
     *
     * @param theta the desired angle in RADIANS
     */
    public void setAngle(double theta) {
        arm.turnToAngle(theta, AngleUnit.RADIANS);
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
                in();
            case IN:
                in();
                break;
            case TRANSIT_OUT:
                out();
                if (elapsedTime.seconds() > getState().time) {
                    setState(Arm.State.OUT);
                }
                break;
            case OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Arm.State.IDLE);
                }
                break;
            case IDLE:
                if (elapsedTime.seconds() > getState().time) {
                    setState(Arm.State.TRANSIT_IN);
                }
                break;
        }
    }
    private void out() {
        setAngle(216.0);
    }

    /**
     * Return platform to rest
     */
    private void in() {
        setAngle(0.0);
    }
    public void cap(){
        setState(Arm.State.TRANSIT_OUT);
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
