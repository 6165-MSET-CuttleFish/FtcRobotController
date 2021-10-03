package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Frontal mechanism for collecting freight
 *
 * @author Bill Yuan
 */
public class Intake extends Module<Intake.State> {
    enum State {
        INTAKING,
        EXTAKING,
        OFF
    }

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void init() {
        // TODO: initialize hardware modules
    }

    @Override
    public State getState() {
        // TODO: return the state of the module
        return null;
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

    @Override
    public void update() {
        // TODO: update control loops and modules
    }
}
