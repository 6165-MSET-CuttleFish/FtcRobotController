package org.firstinspires.ftc.teamcode.util.field

import org.firstinspires.ftc.teamcode.util.field.OpModeType
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry

object Details {
    @JvmField
    var robotPose = Pose2d()
    @JvmField
    var poseVelocity = Pose2d()
    @JvmField
    var alliance = Alliance.NONE
    @JvmField
    var side = Side.NONE
    @JvmField
    var opModeType = OpModeType.NONE
    @JvmField
    var packet = TelemetryPacket()
    @JvmField
    var telemetry: Telemetry? = null
    val futurePose: Pose2d
        get() = robotPose.plus(poseVelocity)
}