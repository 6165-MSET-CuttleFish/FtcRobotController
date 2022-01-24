package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.wrappers.UltrasonicDistanceSensor;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.Side;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.drive.Robot.frontDistanceSensorOffset;
import static org.firstinspires.ftc.teamcode.drive.Robot.horizontalDistanceSensorOffset;
import static org.firstinspires.ftc.teamcode.util.field.Context.alliance;

public class Relocalizer extends Module<Relocalizer.State> {
    private final UltrasonicDistanceSensor frontLeftDistance, frontRightDistance, leftDistance, rightDistance;
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
        frontLeftDistance = new UltrasonicDistanceSensor(hardwareMap, "frontLeftDistance");
        frontRightDistance = new UltrasonicDistanceSensor(hardwareMap, "frontRightDistance");
        leftDistance = new UltrasonicDistanceSensor(hardwareMap, "leftDistance");
        rightDistance = new UltrasonicDistanceSensor(hardwareMap, "rightDistance");
    }

    @Override
    public void internalInit() {

    }

    @Override
    protected void internalUpdate() {
        if (Context.side == Side.CAROUSEL) {

        } else {
            double frontDist = (alliance == Alliance.BLUE ? frontRightDistance.getDistance() : frontLeftDistance.getDistance()) * Math.cos(getTilt());
            double horizontalDist = (alliance == Alliance.BLUE ? leftDistance.getDistance() : rightDistance.getDistance()) * Math.cos(getPitch());
            double frontWallX = 70.5;
            double sideWallY = alliance == Alliance.BLUE ? 70.5 : -70.5;
            double heading = Context.robotPose.getHeading();
            double x = frontWallX - frontDist * Math.cos(heading) - frontDistanceSensorOffset;
            double y = sideWallY - horizontalDist * Math.cos(heading) - horizontalDistanceSensorOffset;
            poseEstimate = new Pose2d(x, y, heading);
        }
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
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
