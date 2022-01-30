package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

public class Capstone extends Module <Capstone.State> {
    public Claw capstoneClaw;
    public Arm capstoneArm;

    @Override
    public boolean isTransitioningState() {
        return false;
    }

    public enum State implements StateBuilder {
        READY,
        PICKING_UP,
        HOLDING,
        PRECAP,
        CAPPING;

        @Override
        public Double getTimeOut() {
            return null;
        }
    }

    /**
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     */
    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.HOLDING);
    }

    public void internalInit() {
        capstoneClaw = new Claw(hardwareMap);
        capstoneArm = new Arm(hardwareMap);
        setNestedModules(capstoneArm, capstoneClaw);
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
                            if (capstoneClaw.getState() == Claw.State.OUT) {
                                capstoneArm.hold();
                            }
                            capstoneClaw.pickUp();
                            break;
                        case TRANSIT_IN:
                            capstoneClaw.pickUp();
                            break;
                        case IN:
                            if (capstoneClaw.getState() == Claw.State.IN) {
                                setState(State.HOLDING);
                            }
                            capstoneClaw.dropDown();
                }
                break;
            case HOLDING:
                capstoneArm.hold();
                if (capstoneArm.getState() == Arm.State.IN) {
                    capstoneClaw.dropDown();
                }
                break;
            case PRECAP:
                switch (capstoneArm.getState()) {
                    case IN:
                        if (capstoneClaw.getState() == Claw.State.OUT) {
                            capstoneArm.preCap();
                        }
                        capstoneClaw.pickUp();
                    case TRANSIT_OUT:
                        break;
                    case OUT:
                        capstoneClaw.half();
                        if (capstoneClaw.getState() == Claw.State.HALF) {
                            setState(State.CAPPING);
                        }
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
    public void pickUp() {
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
    public boolean checker(){
        return (capstoneArm.getState() == Arm.State.IN);
    }
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }
}
