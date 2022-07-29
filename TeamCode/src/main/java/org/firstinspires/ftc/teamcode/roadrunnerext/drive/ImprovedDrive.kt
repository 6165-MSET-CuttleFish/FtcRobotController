package org.firstinspires.ftc.teamcode.roadrunnerext.drive

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.localizers.ImprovedLocalizer
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import java.util.ArrayList

/**
 * Abstraction for generic robot drive motion and localization. Robot poses are specified in a coordinate system with
 * positive x pointing forward, positive y pointing left, and positive heading measured counter-clockwise from the
 * x-axis.
 *
 * @author Ayush Raman
 */
abstract class ImprovedDrive<T>(hardwareMap: HardwareMap) : Module<ImprovedDrive.State>(hardwareMap,
    State.IDLE
) {
    enum class State {
        IDLE,
        BUSY,
    }
    /**
     * Localizer used to determine the evolution of [poseEstimate].
     */
    abstract var localizer: ImprovedLocalizer

    private var headingOffset: Double = 0.0

    /**
     * The raw heading used for computing [externalHeading]. Not affected by [externalHeading] setter.
     */
    protected abstract val rawExternalHeading: Double

    /**
     * The robot's heading in radians as measured by an external sensor (e.g., IMU, gyroscope).
     */
    var externalHeading: Double
        get() = Angle.norm(rawExternalHeading + headingOffset)
        set(value) {
            headingOffset = -rawExternalHeading + value
        }

    /**
     * The robot's current pose estimate.
     */
    var poseEstimate: Pose2d
        get() = localizer.poseEstimate
        set(value) {
            localizer.poseEstimate = value
        }

    /**
     *  Current robot pose velocity (optional)
     */
    val poseVelocity: Pose2d?
        get() = localizer.poseVelocity

    /**
     *  Current robot pose acceleration (optional)
     */
    val poseAcceleration: Pose2d?
        get() = localizer.poseAcceleration

    /**
     * Updates [poseEstimate] with the most recent positional change.
     */
    fun updatePoseEstimate() {
        localizer.update()
    }

    /**
     * Sets the current commanded drive state of the robot. Feedforward is applied to [driveSignal] before it reaches
     * the motors.
     */
    abstract fun setDriveSignal(driveSignal: DriveSignal)

    /**
     * Sets the current commanded drive state of the robot. Feedforward is *not* applied to [drivePower].
     */
    abstract fun setDrivePower(drivePower: Pose2d)

    /**
     * The heading velocity used to determine pose velocity in some cases
     */
    open fun getExternalHeadingVelocity(): Double? = null

    /**
     * The state of the trajectory sequence
     */
    abstract val pathState: T

    abstract fun waitForIdle();

    abstract fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder

    abstract fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder

    abstract fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder

    abstract fun trajectorySequenceBuilder(startPose: Pose2d?): TrajectorySequenceBuilder<*>

    abstract fun turnAsync(angle: Double)

    abstract fun turn(angle: Double)

    abstract fun followTrajectoryAsync(trajectory: Trajectory)

    abstract fun followTrajectory(trajectory: Trajectory)

    abstract fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence)

    abstract fun followTrajectorySequence(trajectorySequence: TrajectorySequence)

    abstract fun setPIDFCoefficients(runMode: DcMotor.RunMode, coefficients: PIDFCoefficients)

    abstract fun setMode(runMode: DcMotor.RunMode)

    abstract fun getWheelPositions(): List<Double>

    abstract fun getWheelVelocities(): List<Double>

    abstract fun setWeightedDrivePower(pose2d: Pose2d)
}
