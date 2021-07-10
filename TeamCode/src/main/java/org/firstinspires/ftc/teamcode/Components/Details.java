package org.firstinspires.ftc.teamcode.Components;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Details {
    public static Pose2d robotPose = new Pose2d();
    public static Side side = Side.RED;
    public static OpModeType opModeType = OpModeType.NONE;
    public static TelemetryPacket packet = new TelemetryPacket();
}
