package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.FrequentPositions
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.OpModeType

@Autonomous
@Disabled
class SampleAutonomous : LinearOpMode() {
    enum class PathState { // state machine for path following
        COLLECTING,
        SCORING,
        IDLE,
    }
    companion object {
        @JvmField var depositDistance = 4.0
        @JvmField var depositingAngle = 60.0
        @JvmField var depositOffset = 2.0
    }
    lateinit var robot: Robot<PathState>
    lateinit var module: Module<*>
    private val blue = true
    override fun runOpMode() {
        robot = Robot(
            this,
            FrequentPositions.startingPosition(),
            OpModeType.AUTO,
            Alliance.RED
        )
        val driveTrain = robot.driveTrain
        // initialization process is slightly inefficient
        // TODO: make the initialization process use less lines of code ig
        val trajectoryBuilder: TrajectorySequenceBuilder<PathState> =
            driveTrain.trajectorySequenceBuilder(FrequentPositions.startingPosition())
                .setReversed(true)
                .setState(PathState.SCORING)
                .foo(module) // custom trajectory builder function
                //.setState(PathState.IDLE)
                // use of a custom trajectory builder function removes ability to 'setState' for that instance
                .splineTo(
                    FrequentPositions.allianceHub.center.polarAdd(
                        depositDistance,
                        Math.toRadians(depositingAngle).flip(blue),
                    ),
                    FrequentPositions.allianceHub.center,
                    Pose2d(0.0, 0.0, Math.toRadians(depositOffset).flip(blue), // position and angle offset (ideally should not be needed if path follower is good)
                    ),
                )
                .setReversed(false)
                .waitWhile(module::isTransitioningState)
                .waitWhile(module::isBusy) // wait for platform to dumpPosition
                .waitWhile(this::waitWhileMotion) {
                    false
                }
                .setReversed(false) as TrajectorySequenceBuilder<PathState>
        // TODO: remove the need for typecasting
        val sequence = trajectoryBuilder
            .setState(PathState.COLLECTING)
            .build()
        driveTrain.followTrajectorySequenceAsync(sequence)
        while (opModeIsActive() && driveTrain.isBusy) {
            robot.update()
            when (robot.driveTrain.pathState) {
                PathState.IDLE -> {}
                PathState.COLLECTING -> {}
                PathState.SCORING -> {}
                else -> {} // should not be needed once path state logic is perfected
            }
        }
    }

    /**
     * @param t time in seconds
     */
    private fun waitWhileMotion(t: Double) : DriveSignal {
        return DriveSignal(Pose2d(t * 2))
    }

}