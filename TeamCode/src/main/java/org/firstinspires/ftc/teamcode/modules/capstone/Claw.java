package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Module to collect the team marker at the start of the match
 * @author Bill Yuan
 */
public class Claw extends Module<Claw.State> {
    enum State {

    }
    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Claw(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {

    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {

    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

}
