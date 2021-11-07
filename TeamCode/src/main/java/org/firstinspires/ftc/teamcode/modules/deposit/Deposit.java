package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.Details;

/**
 * @author Sreyash, Martin
 */
@Config
public class Deposit extends Module<Deposit.State> {
    public enum State {
        LEVEL3(11.75), //tilted 11
        LEVEL2(4), //tilted 7
        IDLE(0.5);
        // MANUAL(0);
        final double dist;
        State(double dist) {
            this.dist = dist;
        }
    }
    DcMotorEx slides;
    public Platform platform;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0.8,0,0.01);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    PIDFController pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);

    double lastKv = kV;
    double lastKa = kA;
    double lastKStatic = kStatic;
    double lastKp = MOTOR_PID.kP;
    double lastKi = MOTOR_PID.kI;;
    double lastKd = MOTOR_PID.kD;
    Telemetry data;
    double powerValue;
    public static double TICKS_PER_INCH = 61.379;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, State.IDLE);
        pidController.setOutputBounds(-1, 1);
        data = telemetry;
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platform = new Platform(hardwareMap);
        slides = hardwareMap.get(DcMotorEx.class, "depositSlides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getPower() {
        return powerValue;
    }

    public void setPower(double power) {
        // setState(State.MANUAL);
        powerValue = power;
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        platform.update(); // update subsystems
        pidController.setTargetPosition(getState().dist);
        if (getState() == State.IDLE) {
            if (elapsedTime.seconds() > 0.5) {
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slides.setPower(0);
                return;
            }
            platform.setState(Platform.State.IN);
            // set power to 0 if error is close to 0
        }
        double power = pidController.update(ticksToInches(slides.getCurrentPosition()));

        slides.setPower(power);

        // for dashboard
        if (kV != lastKv || kA != lastKa || kStatic != lastKStatic || MOTOR_PID.kP != lastKp || MOTOR_PID.kI != lastKi || MOTOR_PID.kD != lastKd) {
            lastKv = kV;
            lastKa = kA;
            lastKStatic = kStatic;
            lastKp = MOTOR_PID.kP;
            lastKi = MOTOR_PID.kI;
            lastKd = MOTOR_PID.kD;

            pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);
        }
        Details.packet.put("Target Height: ", getState().dist);
        Details.packet.put("Actual Height: ", ticksToInches(slides.getCurrentPosition()));
        Details.packet.put("Power: ", power);
        Details.packet.put("Elapsed Time", elapsedTime.seconds());
        Details.packet.put("Motor Current", slides.getCurrent(CurrentUnit.MILLIAMPS));
        Details.packet.put("Velocity", slides.getVelocity());

        data.addData("Target Height: ", getState().dist);
        data.addData("Actual Height: ", ticksToInches(slides.getCurrentPosition()));

        data.update();

    }

    // convert motor ticks to inches traveled by the slides
    public static double ticksToInches(double ticks) {
        // TODO: return inches traveled by slides
        // 145.1 ticks per rev
        return ticks/TICKS_PER_INCH;
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
