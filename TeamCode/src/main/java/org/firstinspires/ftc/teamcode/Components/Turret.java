package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.vision.UGAdvancedHighGoalPipeline;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.TurretTuner;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;

@Config
public class Turret implements Component {
    DcMotorEx turret;
    public static PIDCoefficients ANGLE_PID = new PIDCoefficients( 0.27, 0.0001, 0.002);
    public static PIDCoefficients VISION_PID = new PIDCoefficients(0.0, 0.0, 0.0);
    public static double kV = 1;
    public static double kStatic = 0.01;
    public static double kA = 0.01;
    public double lastKv = kV, lastKp = ANGLE_PID.kP, lastKi = ANGLE_PID.kI, lastKd = ANGLE_PID.kD, lastKStatic = kStatic, lastKa = kA;
    PIDFController angleControl;
    UGAdvancedHighGoalPipeline highGoalPipeline;
    public static double targetAngle = 0;
    VoltageSensor batteryVoltageSensor;
    public Vector2d target;
    TurretTuner turretTuner;
    public static double TICKS_PER_REVOLUTION = 28;
    public static double GEAR_RATIO = (68.0/13.0) * (110.0/24.0);
    public static double TOLERANCE = 0.4;
    private State state = State.IDLE;
    public enum State{
        TARGET_LOCK,
        TUNING,
        IDLE,
    }
    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretTuner = new TurretTuner();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDCoeffecients();
    }
    public void update(){
        double currHeading = Details.robotPose.getHeading();
        Coordinate turretCoord = Coordinate.toPoint(Details.robotPose).polarAdd(currHeading - Math.PI, 4.5);
        double targetAng = 0;
        switch (state){
            case TARGET_LOCK:
                targetAng = getClosestAngle(Math.toDegrees(targetAngle - Details.robotPose.getHeading()));
                if(target != null) targetAng = getClosestAngle(turretCoord.angleTo(Coordinate.toPoint(Robot.goal)) - Math.toDegrees(Details.robotPose.getHeading()));
                break;
            case IDLE:
                targetAng = getClosestZero();
                break;
            case TUNING:
                if(!turretTuner.getRunning()) turretTuner.start();
                targetAng = turretTuner.update();
                break;
        }
        if (targetAng > 400) {
            targetAng -= 360;
        }
        else if (targetAng < -400) {
            targetAng += 360;
        }
        angleControl.setTargetPosition(targetAng);
        double currAngle = Math.toDegrees(getRelativeAngle());
        double power = angleControl.update(currAngle);
        if (getAbsError() < TOLERANCE) {
            turret.setPower(0);
        } else {
            if (getAbsError() < 20) {
            }
         turret.setPower(power);
        }
        if(lastKv != kV || lastKa != kA || lastKStatic != kStatic || lastKp != ANGLE_PID.kP || lastKi != ANGLE_PID.kI || lastKd != ANGLE_PID.kD) {
            lastKv = kV;
            lastKa = kA;
            lastKStatic = kStatic;
            lastKp = ANGLE_PID.kP;
            lastKi = ANGLE_PID.kI;
            lastKd = ANGLE_PID.kD;
            setPIDCoeffecients();
        }
        packet.put("Turret Angle", currAngle);
        packet.put("Turret Velocity", turret.getVelocity());
        packet.put("Target Angle", targetAng);
        DashboardUtil.drawTurret(packet.fieldOverlay(), new Pose2d(turretCoord.getX(), turretCoord.getY(), getAbsoluteAngle()));
    }
    private void setPIDCoeffecients() {
        angleControl = new PIDFController(ANGLE_PID, kV * 12 / batteryVoltageSensor.getVoltage());
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
    private double getClosestAngle(double targetAngle) {
        double curr = Math.toDegrees(getRelativeAngle());
        double option1 = curr > targetAngle ? targetAngle + 360 : targetAngle - 360;
        double range1 = Math.abs(option1 - curr);
        double range2 = Math.abs(targetAngle - curr);
        return range1 < range2 ? option1 : targetAngle;
    }
    private double getClosestZero() {
        double curr = Math.toDegrees(getRelativeAngle());
        double[] possibilities = {-720, -360, 0, 360, 720};
        double minRange = 360;
        int index = 0;
        for (int i = 0; i < possibilities.length; i++) {
            double range = Math.abs(possibilities[i] - curr);
            if (range < minRange) {
                minRange = range;
                index = i;
            }
        }
        return possibilities[index];
    }
    public double getAbsoluteAngle(){
        return Details.robotPose.getHeading() + getRelativeAngle();
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
