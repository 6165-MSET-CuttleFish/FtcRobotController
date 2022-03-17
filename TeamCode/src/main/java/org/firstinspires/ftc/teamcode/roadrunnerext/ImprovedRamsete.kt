package org.firstinspires.ftc.teamcode.roadrunnerext

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.util.NanoClock
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.roadrunnerext.RamseteConstants.*
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Time-varying, non-linear feedback controller for nonholonomic drives. See equation 5.12 of
 * [Ramsete01.pdf](https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf).
 *
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
    private fun sincDeriv(x: Double) =
        if (x epsilonEquals 0.0) {
            -x / 3.0
        } else {
            cos(x) / x - sin(x) / (x * x)
        }

    override fun internalUpdate(currentPose: Pose2d, currentRobotVel: Pose2d?): DriveSignal {
        val currentPose = currentPose.toMeters()
        val t = elapsedTime()
        val targetPose = trajectory[t].toMeters()
        val targetVel = trajectory.velocity(t).toMeters()
        val targetAccel = trajectory.acceleration(t).toMeters()

        val targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel)
        val targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel)

        val targetV = targetRobotVel.x
        val targetA = targetRobotAccel.x
        val targetOmega = targetRobotVel.heading
        val targetAlpha = targetRobotAccel.heading

        //val error = Kinematics.calculateFieldPoseError(targetPose.toInches(), currentPose.toInches()).toMeters()

        val error = Kinematics.calculateRobotPoseError(targetPose, currentPose)

        val k1 = 2 * zeta * sqrt(targetOmega * targetOmega + b * targetV * targetV)
        val k3 = k1
        val k2 = b

        val v = targetV * cos(error.heading) + k1 * error.x
        val omega = targetOmega + k2 * targetV * sinc(error.heading) * error.y + k3 * error.heading

        val outV = v + (currentRobotVel?.toMeters()?.let { kLinear * (v - it.x) } ?: 0.0)

        val outOmega = omega + (currentRobotVel?.toMeters()?.let { kHeading * (omega - it.heading) } ?: 0.0)

        val outputVel = Pose2d(outV, 0.0, outOmega)

        val k1Dot = if (v epsilonEquals 0.0 && omega epsilonEquals 0.0)
            2 * zeta * sqrt(targetAlpha.pow(2) + b * targetA.pow(2))
        else 2 * zeta * (targetOmega * targetAlpha + b * targetV * targetA) /
                sqrt(targetOmega.pow(2) + b * targetV.pow(2))
        val k3Dot = k1Dot
        val errorDot = Pose2d(error.vec().rotated(-Math.PI / 2) * omega +
                targetVel.vec().rotated(-currentPose.heading), targetVel.heading) - outputVel
        val a = -targetV * sin(error.heading) * errorDot.heading +
                targetA * cos(error.heading) + k1 * errorDot.x + k1Dot * error.x
        val alpha = targetAlpha + k2 * (targetV * sinc(error.heading) * errorDot.y + (targetA * sinc(error.heading) + targetV * sincDeriv(error.heading)
                * errorDot.heading) * error.y) + k3 * errorDot.heading + k3Dot * error.heading
        val outputAccel = Pose2d(a, 0.0, alpha)

        lastError = Kinematics.calculateRobotPoseError(targetPose.toInches(), currentPose.toInches())
        lastVelocityError = currentRobotVel?.toMeters()?.let { Kinematics.calculateRobotPoseError(Pose2d(v, 0.0, omega).toInches(), it.toInches()) }
        return DriveSignal(outputVel.toInches(), outputAccel.toInches())
    }
}
