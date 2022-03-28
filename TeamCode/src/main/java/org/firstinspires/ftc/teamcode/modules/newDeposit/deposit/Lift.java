package org.firstinspires.ftc.teamcode.modules.newDeposit.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.vision.ShippingHubDetector;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableMotor;
import org.firstinspires.ftc.teamcode.util.controllers.BPIDFController;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.util.field.Context.balance;
import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;

/**
 * Slides that go up to the level for depositing freight
 * @author Ayush Raman
 */
@Config
public class Lift extends Module<Lift.Level> {
    public static double LEVEL3 = 11.8;
    public static double LEVEL2 = 0;
    public static double LEVEL1 = 0;
    public static boolean resetEncoder = false;
    public static double allowableDepositError = 1;
    public static double angle = Math.toRadians(30);
    ShippingHubDetector hubDetector;

    private double getLevelHeight(Level state) {
        switch (state) {
            case LEVEL3: return LEVEL3;
            case LEVEL2: return LEVEL2;
            case LEVEL1: return LEVEL1;
        }
        return 0;
    }

    public enum Level implements StateBuilder {
        LEVEL3(12),
        LEVEL2(6),
        LEVEL1(0),
        IDLE(0);
        private final double dist;
        Level(double dist) {
            this.dist = dist;
        }
        public double getDist() {
            switch (balance) {
                case BALANCED: return dist;
                case AWAY: return dist + 2;
                case TOWARD: return dist - 1;
            }
            return dist;
        }
        @Override
        public Double getTimeOut() {
            return null;
        }
    }

    ControllableMotor slides;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0.7,0,0.01);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    BPIDFController pidController = new BPIDFController(MOTOR_PID, kV, kA, kStatic);

    double lastKv = kV;
    double lastKa = kA;
    double lastKStatic = kStatic;
    double lastKp = MOTOR_PID.kP;
    double lastKi = MOTOR_PID.kI;
    double lastKd = MOTOR_PID.kD;
    public static double TICKS_PER_INCH = 43.93;

    /**
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Lift(HardwareMap hardwareMap) {
        super(hardwareMap, Level.IDLE, new Pose2d());
        pidController.setOutputBounds(-1, 1);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        slides = new ControllableMotor(hardwareMap.get(DcMotorEx.class, "depositSlides"));
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        if (opModeType != OpModeType.TELE) slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setActuators(slides);
    }

    @Override
    public boolean isTransitioningState() {
        return Math.abs(getLastError()) > allowableDepositError;
    }

    @Override
    public void setState(@NonNull Level state) {
        super.setState(state);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void internalUpdate() {
        Level state = getState();
        pidController.setTargetPosition(getLevelHeight(state));
        double power =  pidController.update(ticksToInches(slides.getCurrentPosition()));
        if (getState() == Level.IDLE && getSecondsSpentInState() < 2.7 && resetEncoder) {
            if (getSecondsSpentInState() > 2.5) { // anti-stall code
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                power = 0.0;
            } else if (getSecondsSpentInState() <= 2.5) {
                power = -1;
            }
        }
        slides.setPower(power);
        // for dashboard
        if (kV != lastKv || kA != lastKa || kStatic != lastKStatic || MOTOR_PID.kP != lastKp || MOTOR_PID.kI != lastKi || MOTOR_PID.kD != lastKd) {
            lastKv = kV;
            lastKa = kA;
            lastKStatic = kStatic;
            lastKp = MOTOR_PID.kP;
            lastKi = MOTOR_PID.kI;
            lastKd = MOTOR_PID.kD;
            pidController = new BPIDFController(MOTOR_PID, kV, kA, kStatic);
        }
        if (isDebugMode()) {
            Context.packet.put("Target Height", pidController.getTargetPosition());
            Context.packet.put("Actual Height", ticksToInches(slides.getCurrentPosition()));
            Context.packet.put("Deposit Motor Power", power);
            Context.packet.put("Lift Error", getLastError());
        }
    }

    public double getLastError() {
        return pidController.getLastError();
    }

    /**
     * @param ticks current position of the motor
     * @return inches traveled by the slides
     */
    private static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    public boolean isDoingInternalWork() {
        return false;
    }
}