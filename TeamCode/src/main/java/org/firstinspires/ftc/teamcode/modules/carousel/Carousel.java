package org.firstinspires.ftc.teamcode.modules.carousel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

import static org.firstinspires.ftc.teamcode.util.field.Details.opModeType;

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

    public void on() {
        double power = opModeType == OpModeType.AUTO ? 0.8 : 1;
        duckyR.setPower(power);
        duckyL.setPower(power);
        setState(State.ON);
    }

    public void off() {
        duckyR.setPower(0);
        duckyL.setPower(0);
        setState(State.IDLE);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
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
