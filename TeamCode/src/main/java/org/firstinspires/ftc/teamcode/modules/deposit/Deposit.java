package org.firstinspires.ftc.teamcode.modules.deposit;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * @author Martin Xu
 */
public class Deposit extends Module<Deposit.State> {
    public enum State {
        LEVEL3(3, 6),
        LEVEL2(2, 4),
        LEVEL1(1, 2),
        IDLE(0, 0);
        final double dist;
        final double time;
        State(double dist, double time) {
            this.dist = dist;
            this.time = time;

        }
    }
    DcMotorEx slides;
    Platform platform;
    StateMachine<Integer> stateMachine;

    public static PIDCoefficients MOTOR_PID = new PIDCoefficients(0,0,0);
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

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Deposit(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
        pidController.setInputBounds(-1, 1);
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

            pidController = new PIDFController(MOTOR_PID, kV, kA, kStatic);
        }
        
    }

    // convert motor ticks to inches traveled by the slides
    private double ticksToInches(double ticks) {
        // TODO: return inches traveled by slides
        return 0;
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
