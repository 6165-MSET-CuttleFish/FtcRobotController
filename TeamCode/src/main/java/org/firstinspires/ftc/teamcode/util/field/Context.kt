package org.firstinspires.ftc.teamcode.util.field

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.toPose

@Config
object Context {
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
        get() = robotPose
            .polarAdd(poseVelocity.x)
            .vec()
            .polarAdd(poseVelocity.y, robotPose.heading + Math.PI)
            .toPose(robotPose.heading)
    @JvmField
    var freight = Freight.NONE
    @JvmField
    var pitch = 0.0
    @JvmField
    var tilt = 0.0
    @JvmField
    var totalCurrent = 0.0
    @JvmField
    var totalCharge = 0.0
    @JvmStatic
    fun reset() {
        robotPose = Pose2d()
        poseVelocity = Pose2d()
        alliance = Alliance.NONE
        side = Side.NONE
        opModeType = OpModeType.NONE
        packet = TelemetryPacket()
        telemetry = null
        freight = Freight.NONE
        pitch = 0.0
        tilt = 0.0
        totalCharge = 0.0
        totalCurrent = 0.0
    }
}