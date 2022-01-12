package org.firstinspires.ftc.teamcode.modules.carousel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

import static org.firstinspires.ftc.teamcode.util.field.Context.alliance;
import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;

/**
 * Mechanism at the back of the robot to deposit freight
 * @author Apoorva Talwalkar
 */
public class Carousel extends Module<Carousel.State> {
    public enum State implements StateBuilder {
        ON,
        IDLE;

        @Override
        public double getTimeOut() {
            return 0;
        }

        @Override
        public double getPercentMotion() {
            return 0;
        }
    }

    CRServo ducky;

    /**
     * Constructor which calls the 'init' function
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Carousel(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        ducky = hardwareMap.crservo.get("duckyR");
        ducky.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        ducky.setPower(power);
    }

    public void on() {
        double power = opModeType == OpModeType.AUTO ? 0.75 : 1;
        if (alliance == Alliance.BLUE) power = -power;
        ducky.setPower(power);
        setState(State.ON);
    }

    public void off() {
        ducky.setPower(0);
        setState(State.IDLE);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    protected void internalUpdate() {
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingInternalWork() {
        return false;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }
}
