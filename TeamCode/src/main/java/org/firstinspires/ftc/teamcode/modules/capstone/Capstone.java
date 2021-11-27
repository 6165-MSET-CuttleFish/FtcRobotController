package org.firstinspires.ftc.teamcode.modules.capstone;

import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;

public class Capstone extends Module <Capstone.State> {
    Slides capstoneSlides;
    Arm capstoneArm;
    public enum State {
        PICKING_UP(0.5),
        HOLDING(0.5),
        CAPPING(0.5);
        final double time;
        State(double time) {
            this.time=time;
        }
    }

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     * @param initialState
     */
    public Capstone(HardwareMap hardwareMap, Object initialState) {
        super(hardwareMap, State.PICKING_UP);
    }

    public void init() {
        capstoneSlides=new Slides(capstoneSlides.hardwareMap);
        capstoneArm=new Arm(capstoneArm.hardwareMap);
        capstoneArm.init();
        capstoneSlides.init();
    }

    @Override
    public void update() {
        switch (getState()) {
            case PICKING_UP:
                capstoneArm.ready();
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.HOLDING);
                }
                break;
            case HOLDING:
                capstoneSlides.pickUp();
                if(capstoneArm.getState()== Arm.State.TRANSIT_OUT){
                    capstoneArm.hold();
                }
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.CAPPING);
                }
                break;
            case CAPPING:
                capstoneSlides.cap();
                if(capstoneArm.getState()== Arm.State.TRANSIT_OUT){
                    capstoneArm.hold();
                }
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.PICKING_UP);
                }
                break;
        }
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    public void ready(){
        setState(State.PICKING_UP);
    }
    public void pickUp(){
        setState(State.HOLDING);
    }
    public void cap(){
        setState(State.CAPPING);
    }
    public boolean isDoingWork() {
        if (capstoneSlides.isDoingWork()||capstoneArm.isDoingWork()){
            return true;
        }
        else
            return false;
    }

    @Override
    public boolean isHazardous() {
        return false;
    }
}
