package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Details {
    public static Pose2d robotPose = new Pose2d();
    public static Pose2d poseVelocity = new Pose2d();
    public static Alliance alliance = Alliance.NONE;
    public static Side side = Side.NONE;
    public static OpModeType opModeType = OpModeType.NONE;
    public static TelemetryPacket packet = new TelemetryPacket();
    public static Telemetry telemetry;
    public static Pose2d getFuturePose() {
        return robotPose.plus(poseVelocity);
    }
}
