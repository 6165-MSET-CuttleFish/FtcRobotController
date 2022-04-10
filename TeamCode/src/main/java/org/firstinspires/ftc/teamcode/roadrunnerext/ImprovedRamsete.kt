package org.firstinspires.ftc.teamcode.roadrunnerext

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.roadrunnerext.RamseteConstants.*
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.toInches
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.toMeters
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Time-varying, non-linear feedback controller for nonholonomic drives. See equation 5.12 of
 * [Ramsete01.pdf](https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf).
 * Property of the Dick Blick Company
 * @param b b parameter (non-negative)
 * @param zeta zeta parameter (on (0, 1))
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param timeout max time to wait for the error to be admissible
 * @param clock clock
 * @author Ayush Raman
 */
class ImprovedRamsete @JvmOverloads constructor(
    admissibleError: Pose2d = Pose2d(2.0, 2.0, Math.toRadians(5.0)),
    timeout: Double =  0.5,
    clock: NanoClock = NanoClock.system(),
) : ImprovedTrajectoryFollower(admissibleError, timeout, clock) {
    override var lastError: Pose2d = Pose2d()
    override var lastVelocityError: Pose2d? = Pose2d()

    private fun sinc(x: Double) =
        if (x epsilonEquals 0.0) {
            1.0 - x * x / 6.0
        } else {
            sin(x) / x
        }

    override fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal {
        val currentPose = currentPose.toMeters()
        val t = elapsedTime()
        val targetPose = trajectory[t].toMeters()
        val targetVel = trajectory.velocity(t).toMeters()
        val targetAccel = if(t > trajectory.duration()) Pose2d() else trajectory.acceleration(t).toMeters()

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose.toInches(), targetVel.toInches()).toMeters()
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose.toInches(), targetVel.toInches(), targetAccel.toInches()).toMeters()

        val targetV = targetRobotVel.x
        val targetOmega = targetRobotVel.heading

        var error = Kinematics.calculateFieldPoseError(targetPose.toInches(), currentPose.toInches()).toMeters()
//        if (Robot.gainMode == Robot.GainMode.BACKWARD) {
//            error = Pose2d(error.x, 0.0, error.heading)
//        }
        val k1 = 2 * zeta * sqrt(targetOmega.pow(2) + b * targetV.pow(2))
        val k3 = k1
        val k2 = b

        val v = targetV * cos(error.heading) +
                k1 * (cos(currentPose.heading) * error.x + sin(currentPose.heading) * error.y)
        val omega = targetOmega + k2 * targetV * sinc(error.heading) *
                (cos(currentPose.heading) * error.y - sin(currentPose.heading) * error.x) +
                k3 * error.heading

        val outV = v + (currentRobotVel?.toMeters()?.let { kLinear * (v - it.x) } ?: 0.0)

        val outOmega = omega + (currentRobotVel?.toMeters()?.let { kHeading * (omega - it.heading) } ?: 0.0)

        lastError = Kinematics.calculateRobotPoseError(targetPose.toInches(), currentPose.toInches())
        lastVelocityError = currentRobotVel?.toMeters()?.let { Kinematics.calculateRobotPoseError(Pose2d(v, 0.0, omega).toInches(), it.toInches()) }
        return DriveSignal(Pose2d(outV, 0.0, outOmega).toInches(), targetRobotAccel.toInches())
    }
}
