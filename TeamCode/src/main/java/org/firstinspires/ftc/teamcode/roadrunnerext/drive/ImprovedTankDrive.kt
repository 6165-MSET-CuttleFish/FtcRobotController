package org.firstinspires.ftc.teamcode.roadrunnerext.drive

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.DriveConstants.*
import org.firstinspires.ftc.teamcode.localizers.ImprovedLocalizer
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.toPose

/**
 * This class provides the basic functionality of a tank/differential drive using [TankKinematics].
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param voltageSensor voltage sensor from hardware map
 */
abstract class ImprovedTankDrive<T> constructor(
    hardwareMap: HardwareMap,
    private val trackWidth: Double,
    private val voltageSensor: VoltageSensor,
) : ImprovedDrive<T>(hardwareMap) {
    /**
     * Default localizer for tank drives based on the drive encoders and (optionally) a heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class TankLocalizer @JvmOverloads constructor(
        private val drive: ImprovedTankDrive<*>,
        private val useExternalHeading: Boolean = true
    ) : ImprovedLocalizer {
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
        private var lastVel: Pose2d? = Pose2d()
        override var poseAcceleration: Pose2d? = Pose2d()

        private fun odoPoseDelta(wheelPositions: List<Double>, extHeading: Double) : Pose2d {
            if (lastWheelPositions.isEmpty() || wheelPositions.isEmpty()) return Pose2d()
            val wheelDeltas = wheelPositions
                .zip(lastWheelPositions)
                .map { it.first - it.second }
                .toMutableList()
            val robotPoseDelta =
                TankKinematics.wheelToRobotVelocities(wheelDeltas, drive.trackWidth)
            val finalHeadingDelta = if (useExternalHeading) {
                Angle.normDelta(extHeading - lastExtHeading)
            } else {
                robotPoseDelta.heading
            }
            return robotPoseDelta.vec().toPose(finalHeadingDelta)
        }

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            val odoDelta = odoPoseDelta(wheelPositions, extHeading)

            _poseEstimate = Kinematics.relativeOdometryUpdate(
                _poseEstimate,
                odoDelta
            )

            val wheelVelocities = drive.getWheelVelocities()?.toMutableList()
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
            poseAcceleration = lastVel?.let { poseVelocity?.minus(it) }
            lastVel = poseVelocity
        }
    }

    private var voltage = 12.0
    private val voltageTimer = ElapsedTime()
    override var localizer: ImprovedLocalizer = TankLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        if (voltageTimer.seconds() > 0.3) {
            voltage = voltageSensor.voltage
            voltageTimer.reset()
        }
        val velocities = TankKinematics.robotToWheelVelocities(driveSignal.vel, trackWidth)
        val accelerations = TankKinematics.robotToWheelAccelerations(driveSignal.accel, trackWidth)

        val voltageMultiplier = 12 / voltage // voltage scaled feedforward
        val powers = Kinematics.calculateMotorFeedforward(
            velocities,
            accelerations,
            kV,
            kA,
            kStatic
        ).toMutableList()
        setMotorPowers(powers[0] * voltageMultiplier, powers[1] * voltageMultiplier)
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val powers = TankKinematics.robotToWheelVelocities(drivePower, 2.0)
        setMotorPowers(powers[0], powers[1])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(left: Double, right: Double)
}
