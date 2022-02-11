package org.firstinspires.ftc.teamcode.modules.relocalizer

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor.MB1242
import org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor.MB1643
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.alliance
import org.firstinspires.ftc.teamcode.util.field.Side
import kotlin.math.cos

@Config
class Relocalizer(hardwareMap: HardwareMap, private val imu: BNO055IMU) : Module<Relocalizer.State>(hardwareMap, State.IDLE) {
    private val frontLeftDistance = MB1242(hardwareMap.i2cDeviceSynch.get("leftFrontDS"))
    private val frontRightDistance = MB1242(hardwareMap.i2cDeviceSynch.get("rightFrontDS"))
    private val leftDistance = MB1643(hardwareMap,"leftDS")
    private val rightDistance = MB1643(hardwareMap,"rightDS")
    enum class State(override val timeOut: Double? = null) : StateBuilder {
       IDLE
    }
    companion object {
        @JvmField
        var frontDistanceSensorXOffset = 7.5
        @JvmField
        var horizontalDistanceSensorYOffset = 7.5
        @JvmField
        var frontDistanceSensorYOffset = -7.0
        @JvmField
        var horizontalDistanceSensorXOffset = -3.0785
    }
    var poseEstimate = Pose2d()
    private val pitchOffset: Double = pitch
    private val tiltOffset: Double = tilt

    val pitch: Double
        get() = imu.angularOrientation.secondAngle - pitchOffset
    val tilt: Double
        get() = imu.angularOrientation.thirdAngle - tiltOffset

    override fun internalInit() {

    }

    override fun internalUpdate() {
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

            poseEstimate = Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
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
                    .polarAdd(if (alliance == Alliance.BLUE) -horizontalDistanceSensorYOffset else horizontalDistanceSensorYOffset, Math.toRadians(90.0))
            poseEstimate = Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
        }
    }

    override fun isDoingInternalWork() = false

    override fun isTransitioningState() = false
}