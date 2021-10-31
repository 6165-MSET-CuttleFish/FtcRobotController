package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.BPIDFController;
import org.firstinspires.ftc.teamcode.util.Details;

/**
 * @author Sreyash, Martin
 */

public class Deposit extends Module<Deposit.State> {
    enum State {
        LEVEL3(3),
        LEVEL2(2),
        LEVEL1(1),
        IDLE(0);
        //auto lift to top, zero itself
        final double dist;
        State(double dist) {
            this.dist = dist;

        }
    }
    DcMotorEx slides;
    Platform platform;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0.1,0,0);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double integralBand = Double.POSITIVE_INFINITY;
    BPIDFController pidController = new BPIDFController(MOTOR_PID, integralBand, kV, kA, kStatic);

    double lastKv = kV;
    double lastKa = kA;
    double lastKStatic = kStatic;
    double lastIntegralBand = integralBand;
    double lastKp = MOTOR_PID.kP;
    double lastKi = MOTOR_PID.kI;;
    double lastKd = MOTOR_PID.kD;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap) {
        super(hardwareMap);


    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platform = new Platform(hardwareMap);
        slides = hardwareMap.get(DcMotorEx.class, "linearSlide");
        pidController.setInputBounds(-1, 1);
    }

    /**
     * This function updates all necessary controls in a loop
     */

    @Override
    public void update() {
        platform.update(); // update subsystems
        pidController.setTargetPosition(getState().dist);
        if (getState() == State.IDLE) {
            platform.setState(Platform.State.IN);
            // set power to 0 if error is close to 0
        }
        slides.setPower(pidController.update(ticksToInches(slides.getCurrentPosition())));

        // for dashboard
        if (kV != lastKv || kA != lastKa || kStatic != lastKStatic || integralBand != lastIntegralBand || MOTOR_PID.kP != lastKp || MOTOR_PID.kI != lastKi || MOTOR_PID.kD != lastKd) {
            lastKv = kV;
            lastKa = kA;
            lastKStatic = kStatic;
            lastIntegralBand = integralBand;
            lastKp = MOTOR_PID.kP;
            lastKi = MOTOR_PID.kI;
            lastKd = MOTOR_PID.kD;

            pidController = new BPIDFController(MOTOR_PID, integralBand, kV, kA, kStatic);
        }
        Details.packet.put("Target Height: ", getState().dist);
        Details.packet.put("Actual Height: ", ticksToInches(slides.getCurrentPosition()));
    }

    // convert motor ticks to inches traveled by the slides
    private double ticksToInches(double ticks) {
        // TODO: return inches traveled by slides
        // TODO: find out ticks after gear ratios
        return ((ticks / 754.52) * 29.1415926536); /* distance pulley covers per revolution, arc length */
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingWork() {
        if(getState() != State.IDLE){
            return true;
        }
        return false;
    }
}
