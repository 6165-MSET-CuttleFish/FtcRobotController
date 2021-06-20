package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotorEx turret;
    public static PIDCoefficients ANGLE_PID = new PIDCoefficients(0.0002, 0, 0);
    PIDFController angleControl = new PIDFController(ANGLE_PID, 0.00002);
    double targetAngle;
    Robot robot;
    public static final double TICKS_PER_REVOLUTION = 2;
    public static final double GEAR_RATIO = 2;
    public enum State{
        MOVING,
        IDLE
    }
    public Turret(HardwareMap hardwareMap, Robot robot){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        this.robot = robot;
        if(robot.opModeType == OpModeType.auto) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void update(){
        angleControl.setTargetPosition(targetAngle);
        double power = angleControl.update(getAbsoluteAngle());
        turret.setPower(power);
    }
    public State getState(){
        return State.IDLE;
    }
    public double getRelativeAngle(){
        return turret.getCurrentPosition() * (2*Math.PI/(TICKS_PER_REVOLUTION * GEAR_RATIO));
    }
    public double getAbsoluteAngle(){
        return robot.getPoseEstimate().getHeading() + getRelativeAngle();
    }
    public void setTargetAngle(double targetAngle){
        this.targetAngle = targetAngle;
    }
}
