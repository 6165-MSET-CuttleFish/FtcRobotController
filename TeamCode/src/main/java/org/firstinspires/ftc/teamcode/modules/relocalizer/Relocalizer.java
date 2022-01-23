package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.wrappers.UltrasonicDistanceSensor;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Context;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.drive.Robot.frontDistanceSensorOffset;
import static org.firstinspires.ftc.teamcode.drive.Robot.horizontalDistanceSensorOffset;
import static org.firstinspires.ftc.teamcode.util.field.Context.alliance;

public class Relocalizer extends Module<Relocalizer.State> {
    private UltrasonicDistanceSensor frontDistance, leftDistance, rightDistance;
    Pose2d poseEstimate;
    private final BNO055IMU imu;
    private final double pitchOffset;
    private final double tiltOffset;
    public enum State implements StateBuilder {
        RELOCALIZING,
        IDLE;

        @Override
        public double getTimeOut() {
            return 0;
        }

        @Override
        public double getPercentMotion() {
            return 0;
        }
    }
    public Relocalizer(@NonNull HardwareMap hardwareMap, BNO055IMU imu) {
        super(hardwareMap, State.IDLE);
        this.imu = imu;
        pitchOffset = getPitch();
        tiltOffset = getTilt();
    }

    @Override
    public void internalInit() {

    }

    @Override
    protected void internalUpdate() {
        double frontDist = frontDistance.getDistance() * Math.cos(getTilt());
        double horizontalDist = (alliance == Alliance.BLUE ? leftDistance.getDistance() : rightDistance.getDistance()) * Math.cos(getPitch());
        double rightWallX = 70.5;
        double bottomWallY = alliance == Alliance.BLUE ? 70.5 : -70.5;
        double heading = Context.robotPose.getHeading();
        double x = rightWallX - frontDist * Math.cos(heading) - frontDistanceSensorOffset;
        double y =  bottomWallY - horizontalDist * Math.cos(heading) - horizontalDistanceSensorOffset;
        poseEstimate = new Pose2d(x, y, heading);
    }

    private double getPitch() {
        return imu.getAngularOrientation().secondAngle - pitchOffset;
    }

    private double getTilt() {
        return imu.getAngularOrientation().thirdAngle - tiltOffset;
    }

    @Override
    protected boolean isDoingInternalWork() {
        return false;
    }

    @Override
    protected boolean isModuleInternalHazardous() {
        return false;
    }
}
