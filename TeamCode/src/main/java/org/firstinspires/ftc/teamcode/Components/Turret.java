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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.TurretTuner;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Components.Details.packet;
import static org.firstinspires.ftc.teamcode.Components.Details.poseVelocity;
import static org.firstinspires.ftc.teamcode.Components.Details.robotPose;

@Config
public class Turret implements Component {
    DcMotorEx turret;
    public static PIDCoefficients ANGLE_PID = new PIDCoefficients(0.24, 0, 0.0007);
    public static double kV = 0;
    public static double kStatic = 0;
    public static double kA = 0;
    private double lastKv = kV, lastKp = ANGLE_PID.kP, lastKi = ANGLE_PID.kI, lastKd = ANGLE_PID.kD, lastKStatic = kStatic, lastKa = kA;
    PIDFController angleControl;
    private double targetAngle = 0;
    VoltageSensor batteryVoltageSensor;
    public Vector2d target;
    TurretTuner turretTuner;
    public static double TICKS_PER_REVOLUTION = 28;
    public static double GEAR_RATIO = (68.0 / 13.0) * (110.0 / 24.0);
    public static double TOLERANCE = 0;
    private State state = State.IDLE;

    public enum State {
        TARGET_LOCK,
        TUNING,
        IDLE,
    }

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretTuner = new TurretTuner();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDCoeffecients();
    }

    public void update() {
        double currHeading = Details.robotPose.getHeading();
        Coordinate turretCoord = Coordinate.toPoint(Details.robotPose).polarAdd(currHeading - Math.PI, 4.5);
        double targetAng = 0;
        switch (state) {
            case TARGET_LOCK:
                targetAng = getClosestAngle(toDegrees(targetAngle - Details.robotPose.getHeading()));
                if (target != null)
                    targetAng = getClosestAngle(turretCoord.angleTo(Coordinate.toPoint(target)) - toDegrees(Details.robotPose.getHeading()));
                break;
            case IDLE:
                targetAng = getClosestZero();
                break;
            case TUNING:
                if (!turretTuner.getRunning()) turretTuner.start();
                targetAng = turretTuner.update();
                break;
        }
        double upperBound = 400;
        double lowerBound = -400;
//        if(WobbleArm.getState() == WobbleArm.State.MID) {//assuming wobble arm is up
//            if ((MathFunctions.AngleWrap(toRadians(targetAngle)) < Math.PI && MathFunctions.AngleWrap(toRadians(targetAngle)) > 0)) {
//                angleControl.setTargetPosition(getClosestDangerMax());
//            } else {
//                upperBound = getClosestDangerMax();
//                lowerBound = getClosestDangerMin();
//            }
//        }
        if (targetAng > upperBound) {
            targetAng -= 360;
        } else if (targetAng < lowerBound) {
            targetAng += 360;
        }
        angleControl.setTargetPosition(targetAng);
        double currAngle = getRelativeAngle();
        double power = angleControl.update(toDegrees(currAngle));
        if (getAbsError() < TOLERANCE) {
            turret.setPower(0);
        } else {
            turret.setPower(power);
        }
        if (lastKv != kV || lastKa != kA || lastKStatic != kStatic || lastKp != ANGLE_PID.kP || lastKi != ANGLE_PID.kI || lastKd != ANGLE_PID.kD) {
            lastKv = kV;
            lastKa = kA;
            lastKStatic = kStatic;
            lastKp = ANGLE_PID.kP;
            lastKi = ANGLE_PID.kI;
            lastKd = ANGLE_PID.kD;
            setPIDCoeffecients();
        }
        packet.put("Turret Angle", toDegrees(currAngle));
        packet.put("Turret Velocity", turret.getVelocity());
        packet.put("Target Angle", targetAng);
        packet.put("Angle Error", getAbsError());
        DashboardUtil.drawTurret(packet.fieldOverlay(), new Pose2d(turretCoord.getX(), turretCoord.getY(), getAbsoluteAngle()), state == State.TARGET_LOCK);
    }

    private void setPIDCoeffecients() {
        angleControl = new PIDFController(ANGLE_PID, kV * 12 / batteryVoltageSensor.getVoltage(), kA, kStatic);
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public double getRelativeAngle() {
        return ticksToAngle(turret.getCurrentPosition());
    }

    public double ticksToAngle(double ticks) {
        return ticks * (2 * Math.PI / (TICKS_PER_REVOLUTION * GEAR_RATIO));
    }

    public double getAbsoluteAngle() {
        return Details.robotPose.getHeading() + getRelativeAngle();
    }

    public double getAngularVelocity() {
        return ticksToAngle(turret.getVelocity());
    }

    public double angleToTicks(double angle) {
        return angle / (2 * Math.PI) * (TICKS_PER_REVOLUTION * GEAR_RATIO);
    }

    private double getClosestAngle(double targetAngle) {
        double curr = toDegrees(getRelativeAngle());
        double option1 = curr > targetAngle ? targetAngle + 360 : targetAngle - 360;
        double range1 = Math.abs(option1 - curr);
        double range2 = Math.abs(targetAngle - curr);
        return range1 < range2 ? option1 : targetAngle;
    }

    private double getClosestZero() {
        double curr = toDegrees(getRelativeAngle());
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

    private double getClosestDangerMax() {
        double curr = toDegrees(getRelativeAngle());
        double[] possibilities = {50, -310};
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

    private double getClosestDangerMin() {
        double curr = toDegrees(getRelativeAngle());
        double[] possibilities = {170, -190};
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

    public void setTargetAngle(double angle) {
        targetAngle = angle;
        target = null;
        state = State.TARGET_LOCK;
    }

    public void setTarget(Vector2d vector2d) {
        target = vector2d;
        state = State.TARGET_LOCK;
    }

    public double getVelocity() {
        return turret.getVelocity();
    }

    public double getError() {
        return toDegrees(angleControl.getLastError());
    }

    public double getAbsError() {
        return Math.abs(getError());
    }

    public boolean isIdle() {
        return Math.abs(turret.getVelocity()) < 200 && getAbsError() < 2;
    }

    public boolean isOnTarget() {
        if (state != State.TARGET_LOCK) return false;
        return Math.abs(turret.getVelocity()) <= 70 && getAbsError() < 0.7;
    }
}
