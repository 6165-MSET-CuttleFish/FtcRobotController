package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Details;

import static org.firstinspires.ftc.teamcode.util.field.Details.balance;

/**
 * Slides that go up to the level for depositing freight
 * @author Martin
 */
@Config
public class Deposit extends Module<Deposit.State> {
    public enum State {
        LEVEL3(11.4), //tilted 11
        LEVEL2(4), //tilted 7
        IDLE(0);
        final private double dist;
        State(double dist) {
            this.dist = dist;
        }
        public double getDist() {
            switch (balance) {
                case BALANCED: return dist;
                case AWAY: return dist + 2;
                case TOWARD: return Range.clip(dist - 1, -1, 12);
            }
            return dist;
        }
    }
    DcMotorEx slides;
    public Platform platform;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(1,0,0.01);
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
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap, Intake intake) {
        super(hardwareMap, State.IDLE);
        pidController.setOutputBounds(-1, 1);
        platform = new Platform(hardwareMap, intake);
        nestedModules = new Module[]{platform};
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        slides = hardwareMap.get(DcMotorEx.class, "depositSlides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private State defaultState = State.LEVEL3;

    @Override
    public void setState(State state) {
        defaultState = state;
    }

    public void dump() {
       platform.dump();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        platform.update(); // update subsystems
        pidController.setTargetPosition(getState().getDist());
        if (platform.isDoingWork()) {
            super.setState(defaultState);
        } else {
            super.setState(State.IDLE);
        }
        double power = pidController.update(ticksToInches(slides.getCurrentPosition()));
        if (getState() == State.IDLE) {
            power = -1;
            if (elapsedTime.seconds() > 0.8) { // anti-stall code
                slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                power = 0;
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
        Details.packet.put("Target Height", getState().dist);
        Details.packet.put("Actual Height", ticksToInches(slides.getCurrentPosition()));
        Details.packet.put("Lift Power", power);
        Details.packet.put("Elapsed Time", elapsedTime.seconds());
        Details.packet.put("Lift Current", slides.getCurrent(CurrentUnit.MILLIAMPS));
        Details.packet.put("Lift Velocity", slides.getVelocity());
    }

    /**
     * @param ticks current position of the motor
     * @return inches traveled by the slides
     */
    private static double ticksToInches(double ticks) {
        return (ticks/TICKS_PER_INCH);
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
