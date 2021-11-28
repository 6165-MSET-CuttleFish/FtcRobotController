package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

public class Capstone extends Module <Capstone.State> {
    Slides capstoneSlides;
    Arm capstoneArm;
    public enum State {
        PICKING_UP,
        HOLDING,
        CAPPING;
    }

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     * @param initialState
     */
    public Capstone(HardwareMap hardwareMap) {
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
        capstoneArm.update();
        capstoneSlides.update();
        switch (getState()) {
            case PICKING_UP:
                switch (capstoneArm.getState()) {
                    case OUT:
                        capstoneSlides.pickUp();
                        if (capstoneSlides.getState() == Slides.State.OUT) {
                            capstoneArm.hold();
                        }
                        break;
                    case TRANSIT_IN:
                        break;
                    case IN:
                        capstoneSlides.dropDown();
                        if (capstoneSlides.getState() == Slides.State.IN) {
                            setState(State.HOLDING);
                        }
                }
                break;
            case HOLDING:
                //TODO: Implement
                break;
            case CAPPING:
                //TODO
                switch (capstoneArm.getState()) {
                    case OUT:
                        capstoneSlides.pickUp();
                        if (capstoneSlides.getState() == Slides.State.OUT) {
                            capstoneArm.ready();
                        }
                        break;
                    case TRANSIT_IN:
                        break;
                    case IN:
                        capstoneSlides.dropDown();
                        if (capstoneSlides.getState() == Slides.State.IN) {
                            setState(State.HOLDING);
                        }
                        capstoneArm.hold();
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
