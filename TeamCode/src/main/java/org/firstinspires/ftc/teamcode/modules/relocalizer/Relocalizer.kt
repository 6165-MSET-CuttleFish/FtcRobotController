package org.firstinspires.ftc.teamcode.modules.relocalizer

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor.MB1242
import org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor.MB1643
import org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor.UltrasonicDistanceSensor
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.alliance
import org.firstinspires.ftc.teamcode.util.field.Side
import kotlin.math.cos

@Config
class Relocalizer(hardwareMap: HardwareMap, private val imu: BNO055IMU) {
    private val frontLeftDistance =
        MB1242(
            hardwareMap.i2cDeviceSynch.get("leftFrontDS"),
            // Pose2d(8.0, 8.0)
        )
    private val frontRightDistance =
        MB1242(
            hardwareMap.i2cDeviceSynch.get("rightFrontDS"),
            // Pose2d(8.0, 8.0)
        )
    private val leftDistance =
        MB1643(
            hardwareMap,
            "leftDS",
            // Pose2d(-8.0, 8.0)
        )
    private val rightDistance =
        MB1643(
            hardwareMap,
            "rightDS",
            // Pose2d(-8.0, -8.0)
        )
    companion object {
        @JvmField
        var frontDistanceSensorXOffset = 8.0
        @JvmField
        var horizontalDistanceSensorYOffset = 8.0
        @JvmField
        var frontDistanceSensorYOffset = 8.0
        @JvmField
        var horizontalDistanceSensorXOffset = 8.0
    }
    val poseEstimate: Pose2d
        get() {
            val frontDist: Double
            val horizontalDist: Double
            if (Context.side == Side.CAROUSEL) {
                frontDist = (if (alliance == Alliance.BLUE) frontRightDistance.getDistance(DistanceUnit.INCH) else frontLeftDistance.getDistance(DistanceUnit.INCH)) * cos(tilt)
                horizontalDist = (if (alliance == Alliance.BLUE) rightDistance.getDistance(DistanceUnit.INCH) else leftDistance.getDistance(DistanceUnit.INCH)) * cos(pitch)
                val frontWallX = 70.5
                val sideWallY = if (alliance == Alliance.BLUE) 70.5 else -70.5
                val heading = Context.robotPose.heading
                val x = frontWallX - horizontalDist * cos(heading)
                val y = sideWallY - frontDist * cos(heading)
                val xPoseEstimate =
                    Pose2d(x, Context.robotPose.y, heading)
                        .polarAdd(-horizontalDistanceSensorXOffset)
                        .polarAdd(-horizontalDistanceSensorYOffset, Math.toRadians(90.0))
                val yPoseEstimate =
                    Pose2d(Context.robotPose.x, y, heading)
                        .polarAdd(-frontDistanceSensorXOffset)
                        .polarAdd(-frontDistanceSensorYOffset, Math.toRadians(90.0))

                return Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
            } else {
                frontDist = (if (alliance == Alliance.BLUE) frontRightDistance.getDistance(DistanceUnit.INCH) else frontLeftDistance.getDistance(DistanceUnit.INCH)) * cos(tilt)
                horizontalDist = (if (alliance == Alliance.BLUE) leftDistance.getDistance(DistanceUnit.INCH) else rightDistance.getDistance(DistanceUnit.INCH)) * cos(pitch)
                val frontWallX = 70.5
                val sideWallY = if (alliance == Alliance.BLUE) 70.5 else -70.5
                val heading = Context.robotPose.heading
                val x = frontWallX - frontDist * cos(heading)
                val y = sideWallY - horizontalDist * cos(heading)
                val xPoseEstimate =
                    Pose2d(x, Context.robotPose.y, heading)
                        .polarAdd(-frontDistanceSensorXOffset)
                        .polarAdd(-frontDistanceSensorYOffset, Math.toRadians(90.0))
                val yPoseEstimate =
                    Pose2d(Context.robotPose.x, y, heading)
                        .polarAdd(-horizontalDistanceSensorXOffset)
                        .polarAdd(horizontalDistanceSensorYOffset, Math.toRadians(90.0))
                return Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
            }
        }
    private val pitchOffset: Double = pitch
    private val tiltOffset: Double = tilt

    val pitch: Double
        get() = imu.angularOrientation.secondAngle - pitchOffset
    val tilt: Double
        get() = imu.angularOrientation.thirdAngle - tiltOffset
}