package org.firstinspires.ftc.teamcode.roadrunnerext

import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import org.firstinspires.ftc.teamcode.drive.DriveConstants.*
import org.firstinspires.ftc.teamcode.util.field.Details
import kotlin.math.abs
import kotlin.math.cos

/**
 * This class provides the basic functionality of a tank/differential drive using [TankKinematics].
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param voltageSensor voltage sensor from hardware map
 */
abstract class ImprovedTankDrive @JvmOverloads constructor(
    private val trackWidth: Double,
    private val voltageSensor: VoltageSensor,
) : Drive() {

    /**
     * Default localizer for tank drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class TankLocalizer @JvmOverloads constructor(
        private val drive: ImprovedTankDrive,
        private val useExternalHeading: Boolean = true
    ) : Localizer {
        private var _poseEstimate = Pose2d()
        override var poseEstimate: Pose2d
            get() = _poseEstimate
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                if (useExternalHeading) drive.externalHeading = value.heading
                _poseEstimate = value
            }
        override var poseVelocity: Pose2d? = null
            private set
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN
        private val timer = ElapsedTime()
        private var _lastGyroPose = Pose2d()

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            val extVelo = drive.getVelocity().toUnit(DistanceUnit.INCH)
            val extPos = drive.getPosition().toUnit(DistanceUnit.INCH)
            Details.packet.put("Gyro Velocity X", extVelo.xVeloc)
            Details.packet.put("Gyro Velocity Y", extVelo.yVeloc)
            Details.packet.put("Gyro Velocity Z", extVelo.zVeloc)
            Details.packet.put("Gyro Position X", extPos.x)
            Details.packet.put("Gyro Position Y", extPos.y)
            Details.packet.put("Gyro Position Z", extPos.z)
            if (abs(cos(drive.getPitch())) > 5) {
                if (integrateUsingPosition) {
                    // POSITION METHOD
                    val x = extPos.x
                    val y = extPos.y
                    val oldPose =
                        _lastGyroPose.vec().polarAdd(gyroXOffset, lastExtHeading + Math.PI / 2)
                            .polarAdd(
                                gyroYOffset, lastExtHeading
                            ).toPose(lastExtHeading)
                    val newPose =
                        Vector2d(x, y).polarAdd(gyroXOffset, extHeading + Math.PI / 2).polarAdd(
                            gyroYOffset, extHeading
                        ).toPose(extHeading)
                    val deltaPose = newPose - oldPose
                    _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, deltaPose)
                } else {
                    // VELOCITY METHOD
                    val dt = timer.seconds()
                    val dx = extVelo.xVeloc * dt
                    val dy = extVelo.yVeloc * dt
                    val robotPoseDelta = Pose2d(dx, dy, extHeading - lastExtHeading)
                    _poseEstimate = Kinematics.relativeOdometryUpdate(_poseEstimate, robotPoseDelta)
                }
            } else if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
                val robotPoseDelta =
                    TankKinematics.wheelToRobotVelocities(wheelDeltas, drive.trackWidth)
                val finalHeadingDelta = if (useExternalHeading) {
                    Angle.normDelta(extHeading - lastExtHeading)
                } else {
                    robotPoseDelta.heading
                }
                _poseEstimate = Kinematics.relativeOdometryUpdate(
                    _poseEstimate,
                    Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
                )
            }

            val wheelVelocities = drive.getWheelVelocities()
            val extHeadingVel = drive.getExternalHeadingVelocity()
            if (wheelVelocities != null) {
                poseVelocity =
                    TankKinematics.wheelToRobotVelocities(wheelVelocities, drive.trackWidth)
                if (useExternalHeading && extHeadingVel != null) {
                    poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
                }
            }

            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
            _lastGyroPose = Pose2d(extPos.x, extPos.y, extHeading)
            timer.reset()
        }
    }

    private var voltage = 12.0
    private val voltageTimer = ElapsedTime()
    override var localizer: Localizer = TankLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        if (voltageTimer.seconds() > 0.5) {
            voltage = voltageSensor.voltage
            voltageTimer.reset()
        }
        val velocities = TankKinematics.robotToWheelVelocities(driveSignal.vel, trackWidth)
        val accelerations = TankKinematics.robotToWheelAccelerations(driveSignal.accel, trackWidth)

        val voltageMultiplier = 12 / voltage
        var powers = Kinematics.calculateMotorFeedforward(
            velocities,
            accelerations,
            kV * voltageMultiplier,
            kA * voltageMultiplier,
            kStatic * voltageMultiplier
        )
        if (powers[0] < 0 && powers[1] < 0) powers = Kinematics.calculateMotorFeedforward(
            velocities,
            accelerations,
            kVBackward * voltageMultiplier,
            kABackward * voltageMultiplier,
            kStaticBackward * voltageMultiplier
        )
        setMotorPowers(powers[0], powers[1])
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(drivePower, 1.0)
        setMotorPowers(powers[0], powers[1])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(left: Double, right: Double)

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the velocities of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    open fun getWheelVelocities(): List<Double>? = null

    abstract fun getVelocity(): Velocity

    abstract fun getPitch(): Double

    abstract fun getPosition(): Position
}
