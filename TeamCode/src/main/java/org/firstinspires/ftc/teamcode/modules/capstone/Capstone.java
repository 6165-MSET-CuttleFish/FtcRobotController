package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

import androidx.annotation.Nullable;

public class Capstone extends Module <Capstone.State> {
    public Arm capstoneArm;
    public enum State implements StateBuilder {
        READY,
        PICKING_UP,
        HOLDING,
        PRECAP,
        CAPPING;

        @Override
        public double getTimeOut() {
            return 0;
        }

        @Override
        public double getPercentMotion() {
            return 0;
        }
    }

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     */
    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.HOLDING);
    }

    public void internalInit() {
        capstoneArm = new Arm(hardwareMap);
        setNestedModules(capstoneArm);
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
                        capstoneArm.hold();
                        break;
                    case TRANSIT_IN:
                        break;
                    case IN:
                        setState(State.HOLDING);
                }
                break;
            case HOLDING:
                capstoneArm.hold();
                break;
            case PRECAP:
                switch (capstoneArm.getState()) {
                    case IN:
                        capstoneArm.preCap();
                    case TRANSIT_OUT:
                        break;
                    case OUT:
                        setState(State.CAPPING);

                        break;
                    case CAP:
                        capstoneArm.preCap();
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
    public void preCap() {
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
        return false;
    }

    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }
}
