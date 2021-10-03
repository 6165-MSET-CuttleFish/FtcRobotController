package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

public class Deposit extends Module<Deposit.State> {
    enum State {

    }
    DcMotorEx linearSlide;
    Platform platform;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {

    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platform = new Platform(hardwareMap);
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
    }

    /**
     * @return The state of the module
     */
    @Override
    public State getState() {
        return null;
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        return false;
    }
}
