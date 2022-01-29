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
    protected enum State implements StateBuilder {
        TRANSIT_OPEN (1.4),
        OPEN(0.5),
        TRANSIT_CLOSE(0.75),
        CLOSE(0.1);
        final double time;
        State(double time) {
            this.time = time;
        }

        @Override
        public double getTimeOut() {
            return 0;
        }

        @Override
        public double getPercentMotion() {
            return 0;
        }
    }
    Servo claw;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Claw(HardwareMap hardwareMap) {
        super(hardwareMap, State.CLOSE);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        claw = hardwareMap.servo.get("capstoneArm");
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void internalUpdate() {
        switch (getState()) {
            case TRANSIT_CLOSE:
                if (getTimeSpentInState() > getState().time) {
                    setState(State.CLOSE);
                }
            case CLOSE:
                close();
                break;
            case TRANSIT_OPEN:
                if (getTimeSpentInState() > getState().time) {
                    setState(State.OPEN);
                }
            case OPEN:
                open();
                break;
        }
    }

    private void open() {
        claw.setPosition(0);
    }
    /**
     * Return platform to rest
     */
    private void close() {
        claw.setPosition(0.45);
    }

    public void openClaw(){
        if (getState() != State.OPEN) setState(State.TRANSIT_OPEN);
    }
    public void closeClaw(){
        if (getState() != State.CLOSE) setState(State.TRANSIT_CLOSE);
    }
    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isDoingInternalWork() {
        return getState() == State.TRANSIT_CLOSE || getState() == Claw.State.TRANSIT_OPEN;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }

}
