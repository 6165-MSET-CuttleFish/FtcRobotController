package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.VelocityPIDFController;

@Config
public class Turret {
    DcMotorEx turret;
    public static PIDCoefficients ANGLE_PID = new PIDCoefficients(0.0002, 0, 0);
    public static double kV = 0.00002;
    private double lastKv = kV, lastKp = ANGLE_PID.kP, lastKi = ANGLE_PID.kI, lastKd = ANGLE_PID.kD;
    PIDFController angleControl = new PIDFController(ANGLE_PID, kV);
    static double targetAngle;
    //Robot robot;
    public static final double TICKS_PER_REVOLUTION = 2;
    public static final double GEAR_RATIO = 2;
    public enum State{
        MOVING,
        IDLE
    }
    public Turret(HardwareMap hardwareMap){
        //HardwareMap hardwareMap = robot.hardwareMap;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        //this.robot = robot;
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void update(){
        angleControl.setTargetPosition(targetAngle);
        double power = angleControl.update(getAbsoluteAngle());
        turret.setPower(power);
        if(lastKv != kV || lastKp != ANGLE_PID.kP || lastKi != ANGLE_PID.kI || lastKd != ANGLE_PID.kD) {
            lastKv = kV;
            angleControl = new PIDFController(ANGLE_PID, kV);
        }
    }
    public State getState(){
        if(turret.getVelocity() > 1){
            return State.MOVING;
        }
        return State.IDLE;
    }
    public double getRelativeAngle(){
        return turret.getCurrentPosition() * (2*Math.PI/(TICKS_PER_REVOLUTION * GEAR_RATIO));
    }
    public double getAbsoluteAngle(){
        //return robot.getPoseEstimate().getHeading() + getRelativeAngle();
        return getRelativeAngle();
    }
    public void setTargetAngle(double targetAngle){
        this.targetAngle = targetAngle;
    }
}
