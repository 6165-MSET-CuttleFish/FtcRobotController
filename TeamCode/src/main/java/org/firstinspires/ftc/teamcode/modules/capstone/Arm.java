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
public class Arm extends Module<Arm.State> {
    protected enum State implements StateBuilder {
        TRANSIT_IN (1.4),
        IN(0.5),
        TRANSIT_OUT(0.75),
        OUT(0.1),
        PRECAP(0.5),
        CAPOFFSET(0),
        CAP(0.3),
        IDLE(0);
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
    Servo arm, wrist;
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
    public void internalInit() {
        arm = hardwareMap.servo.get("capstoneArm");
        wrist = hardwareMap.servo.get("capstoneWrist");
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void internalUpdate() {
        switch (getState()) {
            case TRANSIT_IN:
                if (getTimeSpentInState() > getState().time) {
                    setState(State.IN);
                }
            case IN:
                in();
                break;
            case TRANSIT_OUT:
                if (getTimeSpentInState() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                out();
                break;
            case PRECAP:
                preCapPos();
                if (getTimeSpentInState() > getState().time) {
                    // setState(State.CAPOFFSET);
                }
            case CAPOFFSET:
                break;
            case CAP:
                capPos();
                if (getTimeSpentInState() > getState().time) {
                    // setState(State.IDLE);
                }
                break;

        }
    }

    private void out() {
        wrist.setPosition(1);
        arm.setPosition(outPos);
    }
    /**
     * Return platform to rest
     */
    private void in() {
        wrist.setPosition(1);
        arm.setPosition(inPos);
    }
    private void preCapPos() {
        arm.setPosition(preCap);
        wrist.setPosition(0);
    }
    public static double preCap = 0.1;
    public static double capPos = 0.075;
    public static double inPos = 0.2;
    public static double outPos = 0;
    private void capPos() {
        arm.setPosition(capPos);
    }

    public void ready(){
        if (getState() != State.OUT) setState(State.TRANSIT_OUT);
    }
    public void hold(){
        if (getState() != State.IN) setState(State.TRANSIT_IN);
    }
    public void preCap(){
        setState(State.PRECAP);
    }
    public void cap(){
        setState(State.CAP);
    }
    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isDoingInternalWork() {
        return getState() == State.TRANSIT_IN || getState() == Arm.State.TRANSIT_OUT;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }

}
