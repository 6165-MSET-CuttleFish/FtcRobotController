package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.TurretTuner;

@Config
public class Turret implements Component {
    DcMotorEx turret;
    public static PIDCoefficients ANGLE_PID = new PIDCoefficients( 0.045, 0, 0.0018);
    public static double kV = 1;
    public double lastKv = kV, lastKp = ANGLE_PID.kP, lastKi = ANGLE_PID.kI, lastKd = ANGLE_PID.kD;
    PIDFController angleControl = new PIDFController(ANGLE_PID, kV);
    public static double targetAngle = 0;
    public Vector2d target;
    TurretTuner turretTuner;
    public static double TICKS_PER_REVOLUTION = 28;
    public static double GEAR_RATIO = (68.0/13.0) * (110.0/24.0);
    private State state = State.IDLE;
    public enum State{
        TARGET_LOCK,
        TUNING,
        IDLE,
    }
    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretTuner = new TurretTuner();
        if(Robot.opModeType != OpModeType.TELE) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update(){
        TelemetryPacket packet = new TelemetryPacket();
        double targetAng = 0;
        switch (state){
            case TARGET_LOCK:
                targetAng = Math.toDegrees(MathFunctions.AngleWrap(targetAngle - Robot.robotPose.getHeading()));
                if(target != null) targetAng = Math.toDegrees(MathFunctions.AngleWrap(Robot.robotPose.vec().angleBetween(target) - Robot.robotPose.getHeading()));
                break;
            case IDLE:
                targetAng = 0;
                break;
            case TUNING:
                if(!turretTuner.getRunning()) turretTuner.start();
                targetAng = turretTuner.update();
                break;
        }
        angleControl.setTargetPosition(targetAng);
        double currAngle = Math.toDegrees(getRelativeAngle());
        double power = angleControl.update(currAngle);
        turret.setPower(power);
        if(lastKv != kV || lastKp != ANGLE_PID.kP || lastKi != ANGLE_PID.kI || lastKd != ANGLE_PID.kD) {
            lastKv = kV;
            lastKp = ANGLE_PID.kP;
            lastKi = ANGLE_PID.kI;
            lastKd = ANGLE_PID.kD;
            angleControl = new PIDFController(ANGLE_PID, kV);
        }
        packet.put("Turret Angle", currAngle);
        packet.put("Target Angle", targetAng);
        DashboardUtil.drawTurret(packet.fieldOverlay(), new Pose2d(Robot.robotPose.getX(), Robot.robotPose.getY(), getAbsoluteAngle()));
        dashboard.sendTelemetryPacket(packet);
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
