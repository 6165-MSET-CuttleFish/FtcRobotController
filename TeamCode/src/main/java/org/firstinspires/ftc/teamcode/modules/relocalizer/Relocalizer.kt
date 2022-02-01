package org.firstinspires.ftc.teamcode.modules.relocalizer

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.wrappers.UltrasonicDistanceSensor
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.alliance
import org.firstinspires.ftc.teamcode.util.field.Side
import kotlin.math.cos

@Config
class Relocalizer(hardwareMap: HardwareMap, private val imu: BNO055IMU) : Module<Relocalizer.State>(hardwareMap, State.IDLE) {
    private val frontLeftDistance = UltrasonicDistanceSensor(hardwareMap, "frontLeftDistance")
    private val frontRightDistance = UltrasonicDistanceSensor(hardwareMap, "frontRightDistance")
    private val leftDistance = UltrasonicDistanceSensor(hardwareMap, "leftDistance")
    private val rightDistance = UltrasonicDistanceSensor(hardwareMap, "rightDistance")
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
    var poseEstimate = Pose2d()
    private val pitchOffset: Double = pitch
    private val tiltOffset: Double = tilt

    enum class State : StateBuilder {
        RELOCALIZING, IDLE;

        override val timeOut = 0.0
    }

    override fun internalUpdate() {
        val frontDist: Double
        val horizontalDist: Double
        if (Context.side == Side.CAROUSEL) {
            frontDist = 0.0
            horizontalDist = 0.0
        } else {
            frontDist =
                (if (alliance == Alliance.BLUE) frontRightDistance.distance else frontLeftDistance.distance) * cos(
                    tilt
                )
            horizontalDist =
                (if (alliance == Alliance.BLUE) leftDistance.distance else rightDistance.distance) * cos(
                    pitch
                )
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
                    .polarAdd(-horizontalDistanceSensorYOffset, Math.toRadians(90.0))
            poseEstimate = Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
        }
        Context.packet.put("Front Distance", frontDist)
        Context.packet.put("Front Distance", horizontalDist)
    }

    private val pitch: Double
        get() = imu.angularOrientation.thirdAngle - pitchOffset
    private val tilt: Double
        get() = imu.angularOrientation.secondAngle - tiltOffset

    override fun isDoingInternalWork(): Boolean {
        return false
    }

    override fun isModuleInternalHazardous(): Boolean {
        return false
    }

    override fun internalInit() {
        setNestedModules(frontLeftDistance, frontRightDistance, rightDistance, leftDistance)
    }
    override fun isTransitioningState(): Boolean = false
}