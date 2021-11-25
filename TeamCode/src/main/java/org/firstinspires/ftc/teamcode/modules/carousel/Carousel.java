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
    public enum State {
        ON,
        IDLE,
    }

    CRServo duckyR, duckyL;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Carousel(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        duckyR = hardwareMap.crservo.get("duckyR");
        duckyL = hardwareMap.crservo.get("duckyL");
        duckyL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     *
     */
    public void setState(State state) {
        super.setState(state);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {
            case ON:
                duckyR.setPower(0.8);
                duckyL.setPower(0.8);
                break;
            case IDLE:
                duckyR.setPower(0);
                duckyL.setPower(0);
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
