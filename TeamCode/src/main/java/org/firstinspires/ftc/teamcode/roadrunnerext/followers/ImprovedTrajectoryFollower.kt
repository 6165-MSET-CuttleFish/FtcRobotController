package org.firstinspires.ftc.teamcode.roadrunnerext.followers

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock
import org.firstinspires.ftc.teamcode.Robot
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Generic [Trajectory] follower for time-based pose reference tracking.
 *
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 */
abstract class ImprovedTrajectoryFollower @JvmOverloads constructor(
    var admissibleError: Pose2d = Pose2d(),
    var timeout: Double = 0.0,
    var admissibleVelo: Pose2d = Pose2d(),
    private val clock: NanoClock = NanoClock.system(),
) {
    private var startTimestamp: Double = 0.0
    private var admissible = false
    private var remainingMarkers = mutableListOf<TrajectoryMarker>()
    private var executedFinalUpdate = false

    /**
     * Trajectory being followed if [isFollowing] is true.
     */
    lateinit var trajectory: Trajectory
        protected set

    /**
     * Robot pose error computed in the last [update] call.
     */
    abstract var lastError: Pose2d
        protected set

    /**
     * Robot velocity error computed in the last [update] call.
     */
    abstract var lastVelocityError: Pose2d?
        protected set

    /**
     * Follow the given [trajectory].
     */
    @JvmOverloads open fun followTrajectory(trajectory: Trajectory, offset: Double = 0.0) {
        this.startTimestamp = clock.seconds() - offset
        this.trajectory = trajectory
        this.admissible = false

        remainingMarkers.clear()
        remainingMarkers.addAll(trajectory.markers)
        remainingMarkers.sortBy { it.time }

        executedFinalUpdate = false
    }

    private fun internalIsFollowing(): Boolean {
        val timeRemaining = trajectory.duration() - elapsedTime()
        return timeRemaining > 0 || (!admissible && timeRemaining > -timeout)
    }

    /**
     * Returns true if the current trajectory is currently executing.
     */
    fun isFollowing() = !executedFinalUpdate || internalIsFollowing()

    /**
     * Returns the elapsed time since the last [followTrajectory] call.
     */
    fun elapsedTime() = clock.seconds() - startTimestamp

    /**
     * Run a single iteration of the trajectory follower.
     *
     * @param currentPose current field frame pose
     * @param currentRobotVel current robot frame velocity
     */
    @JvmOverloads
    fun update(currentPose: Pose2d, currentRobotVel: Pose2d? = null): DriveSignal {
        if (Robot.admissibleError != this.admissibleError || Robot.admissibleTimeout != this.timeout || Robot.admissibleVelo != admissibleVelo) {
            this.admissibleError = Robot.admissibleError
            this.timeout = Robot.admissibleTimeout
            this.admissibleVelo = Robot.admissibleVelo
        }
        while (remainingMarkers.size > 0 && elapsedTime() > remainingMarkers[0].time) {
            remainingMarkers.removeAt(0).callback.onMarkerReached()
        }

        val trajEndError = trajectory.end() - currentPose
        val signal = internalUpdate(currentPose, currentRobotVel)
        val targetVel = signal.vel
        admissible = hypot(trajEndError.x, trajEndError.y) < admissibleError.x &&
                abs(Angle.normDelta(trajEndError.heading)) < admissibleError.heading ||
                hypot(targetVel.x, targetVel.y) < admissibleVelo.x &&
                        abs(Angle.norm(targetVel.heading)) < admissibleVelo.heading
        return if (internalIsFollowing() || executedFinalUpdate) {
            signal
        } else {
            for (marker in remainingMarkers) {
                marker.callback.onMarkerReached()
            }
            remainingMarkers.clear()
            executedFinalUpdate = true
            DriveSignal()
        }
    }

    protected abstract fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal
}
