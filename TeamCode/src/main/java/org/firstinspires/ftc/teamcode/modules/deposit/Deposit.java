package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Context;

import androidx.annotation.NonNull;
import static org.firstinspires.ftc.teamcode.util.field.Context.balance;

/**
 * Slides that go up to the level for depositing freight
 * @author Martin
 */
@Config
public class Deposit extends Module<Deposit.State> {
    public static boolean farDeposit;
    public static double LEVEL3 = 12.8;
    public static double LEVEL2 = 5;
    public static double LEVEL1 = 0;
    public static double allowableDepositError = 4;

    @Override
    public boolean isTransitioningState() {
        return getLastError() < allowableDepositError;
    }

    public enum State implements StateBuilder {
        LEVEL3(12),
        LEVEL2(5),
        LEVEL1(0),
        IDLE(0);
        private final double dist;
        State(double dist) {
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
        public double getTimeOut() {
            return 0;
        }

        @Override
        public double getPercentMotion() {
            return 0;
        }
    }

    DcMotorEx slides;
    public Platform platform;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0.6,0,0.001);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    PIDFController pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);

    double lastKv = kV;
    double lastKa = kA;
    double lastKStatic = kStatic;
    double lastKp = MOTOR_PID.kP;
    double lastKi = MOTOR_PID.kI;
    double lastKd = MOTOR_PID.kD;
    public static double TICKS_PER_INCH = 43.93;

    /**
     * @param hardwareMap instance of the hardware map provided by the OpMode
     * @param intake instance of robot's intake module
     */
    public Deposit(HardwareMap hardwareMap, Intake intake) {
        super(hardwareMap, State.IDLE, new Pose2d(), 1);
        pidController.setOutputBounds(-1, 1);
        platform = new Platform(hardwareMap, intake, this);
        setNestedModules(platform);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
        slides = hardwareMap.get(DcMotorEx.class, "depositSlides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private State defaultState = State.LEVEL3;

    public State getDefaultState() {
        return defaultState;
    }

    @Override
    public void setState(@NonNull State state) {
        if (state == defaultState) return;
        if (state == State.IDLE) defaultState = State.LEVEL1;
        else defaultState = state;
        if (platform.platformIsOut(platform.getState()) && farDeposit)
            platform.safetyPosition();
    }

    public void dump() {
       platform.dump();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void internalUpdate() {
        pidController.setTargetPosition(farDeposit ? getState().getDist() : 0);
        if (platform.isDoingWork()) {
            super.setState(defaultState);
        } else {
            super.setState(State.IDLE);
        }
        double power =  pidController.update(ticksToInches(slides.getCurrentPosition()));
        if (getState() == State.IDLE) {
            if (getTimeSpentInState() > 0.5) { // anti-stall code
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                power = 0;
            } else if (getTimeSpentInState() <= 0.5) {
                power = -0.7;
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
            pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);
        }
        Context.packet.put("Target Height", pidController.getTargetPosition());
        Context.packet.put("Actual Height", ticksToInches(slides.getCurrentPosition()));
        Context.packet.put("Deposit Motor Power", power);
    }

    public double getLastError() {
        return getState() != State.IDLE ? pidController.getLastError() : (getTimeSpentInState() > 0.5 ? 0 : 100);
    }

    /**
     * @param ticks current position of the motor
     * @return inches traveled by the slides
     */
    private static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }
    
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }
      
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    public boolean isDoingInternalWork() {
        return platform.isDoingInternalWork();
    }
}
