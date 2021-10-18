package org.firstinspires.ftc.teamcode.modules.carousel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Mechanism at the back of the robot to deposit freight
 *
 * @author Apoorva Talwalkar
 */
public class Carousel extends Module<Carousel.State> {
    enum State {
        ON,
        IDLE,
    }

    CRServo driver, driver1;

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
        driver = hardwareMap.crservo.get("driver");
        driver1 = hardwareMap.crservo.get("driver1");
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case ON:
                // TODO: code to turn on drivers
                break;
            case IDLE:
                // TODO: code to turn off drivers
                break;
        }
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingWork() {
        return false;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isHazardous() {
        return false;
    }
}
