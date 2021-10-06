package org.firstinspires.ftc.teamcode.modules.carousel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
/**
 * Mechanism at the back of the robot to deposit freight
 *
 * @author Apoorva Talwalkar
 */
public class Carousel extends Module<Carousel.State> {
    /**
     * States of the Carousel module
     */
    enum State {

    }

    CRServo driver1, driver2;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Carousel(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        driver1 = hardwareMap.crservo.get("driver1");
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {

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
