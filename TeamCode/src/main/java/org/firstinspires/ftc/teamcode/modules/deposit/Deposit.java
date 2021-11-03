package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.Details;

/**
 * @author Sreyash, Martin
 */
public class Deposit extends Module<Deposit.State> {
    public enum State {
        LEVEL3(22.0),
        LEVEL2(13.5),
        LEVEL1(3.0),
        IDLE(0.0);
        final double dist;
        State(double dist) {
            this.dist = dist;
        }
    }
    DcMotorEx slides;
    Platform platform;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(8,0,0);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double integralBand = Double.POSITIVE_INFINITY;
    PIDFController pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);

    double lastKv = kV;
    double lastKa = kA;
    double lastKStatic = kStatic;
    double lastIntegralBand = integralBand;
    double lastKp = MOTOR_PID.kP;
    double lastKi = MOTOR_PID.kI;;
    double lastKd = MOTOR_PID.kD;
    Telemetry data;
    double powerValues;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, State.IDLE);
        pidController.setInputBounds(-1, 1);
        data = telemetry;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platform = new Platform(hardwareMap);
        slides = hardwareMap.get(DcMotorEx.class, "depositSlides");
    }

    /**
     * This function updates all necessary controls in a loop
     */
    public double power() {
        return powerValues;
    }
    @Override
    public void update() {
        //platform.update(); // update subsystems
//        pidController.setTargetPosition(getState().dist);
//        if (getState() == State.IDLE) {
//            platform.setState(Platform.State.IN);
//            // set power to 0 if error is close to 0
//        }
        powerValues = pidController.update(slides.getCurrentPosition());
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

            pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);
        }
        Details.packet.put("Target Height: ", getState().dist);
        Details.packet.put("Actual Height: ", ticksToInches(slides.getCurrentPosition()));

        data.addData("Target Height: ", getState().dist);
        data.addData("Actual Height: ", ticksToInches(slides.getCurrentPosition()));
        data.addData("inches to ticks: ", inchesToTicks(getState().dist));
        data.addData("power ", power());

        data.update();

    }

    // convert motor ticks to inches traveled by the slides
    public static double ticksToInches(double ticks) {
        // TODO: return inches traveled by slides
        return -1 * ((ticks / 754.52) * 29.1415926536); /* distance pulley covers per revolution, arc length */
    }
    public static double inchesToTicks(double inches) {
        return (( inches / 29.1415926536) * 754.52); /* distance pulley covers per revolution, arc length */
    }

    
    @Override
    public boolean isHazardous() {
        return false;
    }
      
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    public boolean isDoingWork() {
        return false;
    }
}
