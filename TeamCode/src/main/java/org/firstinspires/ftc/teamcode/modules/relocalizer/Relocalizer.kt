package org.firstinspires.ftc.teamcode.modules.relocalizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.wrappers.UltrasonicDistanceSensor
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.alliance
import org.firstinspires.ftc.teamcode.util.field.Side
import kotlin.math.cos

class Relocalizer(hardwareMap: HardwareMap, private val imu: BNO055IMU) : Module<Relocalizer.State>(hardwareMap, State.IDLE) {
    private val frontLeftDistance: UltrasonicDistanceSensor
    private val frontRightDistance: UltrasonicDistanceSensor
    private val leftDistance: UltrasonicDistanceSensor
    private val rightDistance: UltrasonicDistanceSensor
    var poseEstimate = Pose2d()
    private val pitchOffset: Double
    private val tiltOffset: Double

    enum class State : StateBuilder {
        RELOCALIZING, IDLE;

        override val timeOut = 0.0

        override val percentMotion = 0.0
    }

    override fun internalInit() {}
    override fun internalUpdate() {
        if (Context.side == Side.CAROUSEL) {
        } else {
            val frontDist =
                (if (alliance == Alliance.BLUE) frontRightDistance.distance else frontLeftDistance.distance) * cos(
                    tilt
                )
            val horizontalDist =
                (if (alliance == Alliance.BLUE) leftDistance.distance else rightDistance.distance) * cos(
                    pitch
                )
            val frontWallX = 70.5
            val sideWallY = if (alliance == Alliance.BLUE) 70.5 else -70.5
            val heading = Context.robotPose.heading
            val x = frontWallX - frontDist * cos(heading) - Robot.frontDistanceSensorOffset
            val y =
                sideWallY - horizontalDist * cos(heading) - Robot.horizontalDistanceSensorOffset
            poseEstimate = Pose2d(x, y, heading)
        }
    }

    private val pitch: Double
        get() = imu.angularOrientation.secondAngle - pitchOffset
    private val tilt: Double
        get() = imu.angularOrientation.thirdAngle - tiltOffset

    override fun isDoingInternalWork(): Boolean {
        return false
    }

    override fun isModuleInternalHazardous(): Boolean {
        return false
    }

    init {
        pitchOffset = pitch
        tiltOffset = tilt
        frontLeftDistance = UltrasonicDistanceSensor(hardwareMap, "frontLeftDistance")
        frontRightDistance = UltrasonicDistanceSensor(hardwareMap, "frontRightDistance")
        leftDistance = UltrasonicDistanceSensor(hardwareMap, "leftDistance")
        rightDistance = UltrasonicDistanceSensor(hardwareMap, "rightDistance")
    }
}