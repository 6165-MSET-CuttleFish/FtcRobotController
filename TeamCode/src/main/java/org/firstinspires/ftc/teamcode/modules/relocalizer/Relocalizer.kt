package org.firstinspires.ftc.teamcode.modules.relocalizer

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor.UltrasonicDistanceSensor
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.alliance
import org.firstinspires.ftc.teamcode.util.field.Side
import javax.lang.model.type.NullType
import kotlin.math.cos
import kotlin.math.sin

@Config
class Relocalizer(hardwareMap: HardwareMap, private val imu: BNO055IMU) : Module<NullType?>(hardwareMap, null) {
    enum class Sensor {
        FRONT_LEFT,
        FRONT_RIGHT,
        LEFT,
        RIGHT
    }
    private val frontLeftDistance =
        UltrasonicDistanceSensor(
            hardwareMap ,
            "leftFrontDS",
            UltrasonicDistanceSensor.SensorType.LongRange,
            Pose2d(7.5, 7.0, Math.toRadians(45.0))
        )
    private val frontRightDistance = UltrasonicDistanceSensor(
        hardwareMap ,
        "rightFrontDS",
        UltrasonicDistanceSensor.SensorType.LongRange,
        Pose2d(7.5, -7.0, Math.toRadians(-45.0))
    )
    private val leftDistance = UltrasonicDistanceSensor(
        hardwareMap ,
        "leftDS",
        UltrasonicDistanceSensor.SensorType.ShortRange,
        Pose2d(-3.0785, 7.5, Math.toRadians(90.0))
    )
    private val rightDistance = UltrasonicDistanceSensor(
        hardwareMap ,
        "rightDS",
        UltrasonicDistanceSensor.SensorType.LongRange,
        Pose2d(-3.0785, -7.5, Math.toRadians(-90.0))
    )
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
        private set

    private val pitchOffset: Double = pitch
    private val tiltOffset: Double = tilt

    private val pitch: Double
        get() = imu.angularOrientation.secondAngle - pitchOffset
    private val tilt: Double
        get() = imu.angularOrientation.thirdAngle - tiltOffset

    override fun internalInit() {}

    private fun getSensor(sensor: Sensor): UltrasonicDistanceSensor {
        return when (sensor) {
            Sensor.FRONT_LEFT -> frontLeftDistance
            Sensor.FRONT_RIGHT -> frontRightDistance
            Sensor.LEFT -> leftDistance
            Sensor.RIGHT -> rightDistance
        }
    }

    fun updatePoseEstimate(xCorrection: Sensor, yCorrection: Sensor) {
        val xSensor = getSensor(xCorrection)
        val ySensor = getSensor(yCorrection)
        val frontWallX = 70.5
        val sideWallY = if (alliance == Alliance.BLUE) 70.5 else -70.5
        val heading = Context.robotPose.heading
        val xDist = xSensor.getDistance(DistanceUnit.INCH) * cos(sin(xSensor.poseOffset.heading) * tilt + cos(xSensor.poseOffset.heading) * pitch)
        val yDist = ySensor.getDistance(DistanceUnit.INCH) * cos(sin(xSensor.poseOffset.heading) * tilt + cos(xSensor.poseOffset.heading) * pitch)
        val x = frontWallX - xDist * (xSensor.modulePoseEstimate.heading)
        val y = sideWallY - yDist * (ySensor.modulePoseEstimate.heading)
        val xPoseEstimate =
            Pose2d(x, Context.robotPose.y, heading)
                .polarAdd(-xSensor.poseOffset.x)
                .polarAdd(-xSensor.poseOffset.y, Math.toRadians(90.0))
        val yPoseEstimate =
            Pose2d(Context.robotPose.x, y, heading)
                .polarAdd(-ySensor.poseOffset.x)
                .polarAdd(-ySensor.poseOffset.y, Math.toRadians(90.0))
        poseEstimate = Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
    }

    override fun internalUpdate() {
        val frontDist: Double
        val horizontalDist: Double
        if (Context.side == Side.CAROUSEL) {
            frontDist = (if (alliance == Alliance.BLUE) frontRightDistance.getDistance(DistanceUnit.INCH) else frontLeftDistance.getDistance(DistanceUnit.INCH)) * cos(pitch)
            horizontalDist = (if (alliance == Alliance.BLUE) rightDistance.getDistance(DistanceUnit.INCH) else leftDistance.getDistance(DistanceUnit.INCH)) * cos(tilt)
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