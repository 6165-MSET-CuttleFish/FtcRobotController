package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

public class Capstone extends Module <Capstone.State> {
    public Slides capstoneSlides;
    public Arm capstoneArm;
    public enum State {
        READY,
        PICKING_UP,
        HOLDING,
        PRECAP,
        CAPPING;
    }

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     */
    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.HOLDING);
    }

    public void init() {
        capstoneSlides = new Slides(hardwareMap);
        capstoneArm = new Arm(hardwareMap);
        setNestedModules(capstoneArm, capstoneSlides);
    }

    @Override
    protected void internalUpdate() {
        switch (getState()) {
            case READY:
                capstoneArm.ready();
                break;
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
                        if (capstoneSlides.getState() == Slides.State.IN) {
                            setState(State.HOLDING);
                        }
                        capstoneSlides.dropDown();
                }
                break;
            case HOLDING:
                capstoneArm.hold();
                if (capstoneArm.getState() == Arm.State.IN) {
                    capstoneSlides.dropDown();
                }
                break;
            case PRECAP:
                switch (capstoneArm.getState()) {
                    case IN:
                        if (capstoneSlides.getState() == Slides.State.OUT) {
                            capstoneArm.precap();
                        }
                        capstoneSlides.pickUp();
                    case TRANSIT_OUT:
                        break;
                    case OUT:
                        capstoneSlides.half();
                        if (capstoneSlides.getState() == Slides.State.HALF) {
                            setState(State.CAPPING);
                        }
                        break;
                    case CAP:
                        capstoneArm.precap();
                        break;
                }
                break;
            case CAPPING:
                capstoneArm.cap();
                break;
        }


    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    public void pickUp(){
        setState(State.PICKING_UP);
    }
    public void precap() {
        setState(State.PRECAP);
    }
    public void cap() {
        if (getState() == State.PRECAP) setState(State.CAPPING);
    }
    public void ready() {
        setState(State.READY);
    }
    public void hold() {
        setState(State.HOLDING);
    }
    public boolean isDoingInternalWork() {
        return capstoneSlides.isDoingInternalWork() || capstoneArm.isDoingInternalWork();
    }

    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }
}
