package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

public class Capstone extends Module <Capstone.State> {
    Slides capstoneSlides;
    Arm capstoneArm;
    public enum State {
        IDLE,
        PICKING_UP,
        HOLDING,
        CAPPING;
    }

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     */
    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
    }

    public void init() {
        capstoneSlides=new Slides(hardwareMap);
        capstoneArm=new Arm(hardwareMap);
        capstoneArm.init();
        capstoneSlides.init();
        nestedModules=new Module[]{capstoneArm,capstoneSlides};
      //  capstoneArm.ready();
    }

    @Override
    public void update() {
        capstoneArm.update();
        capstoneSlides.update();
        switch (getState()) {
            case IDLE:
                capstoneArm.ready();
            case PICKING_UP:
                switch (capstoneArm.getState()) {
                    case OUT:
                        if (capstoneSlides.getState() == Slides.State.OUT) {
                            capstoneArm.hold();
                        }
                        capstoneSlides.pickUp();

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
                    case IN:
                        if (capstoneSlides.getState() == Slides.State.OUT) {
                            capstoneArm.cap();
                        }
                        capstoneSlides.pickUp();
                    case TRANSIT_OUT:
                        break;
                    case OUT:
                        capstoneSlides.half();
                        if (capstoneSlides.getState() == Slides.State.HALF) {
                            setState(State.HOLDING);
                        }
                        break;
                }
                break;
        }

    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    public void pickUp(){
        setState(State.PICKING_UP);
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
