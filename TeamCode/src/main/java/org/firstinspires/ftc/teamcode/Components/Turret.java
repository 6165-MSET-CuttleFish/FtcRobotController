package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    DcMotorEx turret;
    public static PIDCoefficients ANGLE_PID = new PIDCoefficients( 0.028, 0, 0);
    public static double kV = 1;
    public double lastKv = kV, lastKp = ANGLE_PID.kP, lastKi = ANGLE_PID.kI, lastKd = ANGLE_PID.kD;
    PIDFController angleControl = new PIDFController(ANGLE_PID);
    public static double targetAngle = 0;
    public Vector2d target;
    //Robot robot;
    public static double TICKS_PER_REVOLUTION = 28;
    public static double GEAR_RATIO = (68.0/13.0) * (110.0/24.0);
    private State state;
    public enum State{
        TARGET_LOCK,
        IDLE,
    }
    public Turret(HardwareMap hardwareMap){
        //HardwareMap hardwareMap = robot.hardwareMap;
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        //this.robot = robot;
        if(Robot.opModeType != OpModeType.TELE) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update(){
        switch (state){
            case TARGET_LOCK:
                angleControl.setTargetPosition(targetAngle - Robot.robotPose.getHeading());
                if(target != null) angleControl.setTargetPosition(Math.toDegrees(Robot.robotPose.vec().angleBetween(target) - Robot.robotPose.getHeading()));
                break;
            case IDLE:
                angleControl.setTargetPosition(0);
                break;
        }

        double power = angleControl.update(Math.toDegrees(getRelativeAngle()));
        turret.setPower(power);
        if(lastKv != kV || lastKp != ANGLE_PID.kP || lastKi != ANGLE_PID.kI || lastKd != ANGLE_PID.kD) {
            lastKv = kV;
            lastKp = ANGLE_PID.kP;
            lastKi = ANGLE_PID.kI;
            lastKd = ANGLE_PID.kD;
            angleControl = new PIDFController(ANGLE_PID, kV);
        }

    }
    public State getState(){
        return state;
    }
    public void setState(State state){
        this.state = state;
    }
    public double getRelativeAngle(){
        return turret.getCurrentPosition() * (2*Math.PI/(TICKS_PER_REVOLUTION * GEAR_RATIO));
    }
    public double getAbsoluteAngle(){
        return Robot.robotPose.getHeading() + getRelativeAngle();
    }
    public void setTarget(Vector2d vector2d) {
        target = vector2d;
    }
    public double getVelocity() {
        return turret.getVelocity();
    }
    public double getError() {
        return angleControl.getLastError();
    }
    public double getAbsError() {
        return Math.abs(getError());
    }
    public boolean isIdle() {
        return turret.getVelocity() < 100 && getAbsError() < 5;
    }
}
