package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.allianceHub
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.GainMode
import org.firstinspires.ftc.teamcode.drive.Robot.admissibleError
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.relocalizer.Relocalizer
import org.firstinspires.ftc.teamcode.modules.vision.TSEDetector
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context.location
import org.firstinspires.ftc.teamcode.util.field.Context.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import org.firstinspires.ftc.teamcode.util.Async
import org.firstinspires.ftc.teamcode.util.field.Context
import kotlin.Throws
import kotlin.math.sin

@Autonomous
@Config
class CyclingRed : LinearOpMode() {
    //TUNING NOTES:
    // Adjust acceleration, and cycling distance
    lateinit var robot: Robot<PathState>
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    lateinit var relocalizer: Relocalizer
    private val blue = false
    companion object {
        @JvmField var coast = -54.5
        @JvmField var stop = 49.0
        @JvmField var intakeDelay = 2.0
        @JvmField var powerDelay = 2.1
        @JvmField var depositDelay = 17.0
        @JvmField var cycles = 6
        @JvmField var conjoiningPoint = 20.0
        @JvmField var conjoiningDeposit = 30.0
        @JvmField var waitTime = 0.08
        @JvmField var gainsPoint = 36.0
        @JvmField var cyclingDistance = 20.0
        @JvmField var depositDistance = 25.0
        @JvmField var divConstant = 4.0
        @JvmField var depositingAngle = -55.0
        @JvmField var cyclingAngle = -54.0
        @JvmField var depositingTimeout = 0.4
        @JvmField var intakeError = 8.0
        @JvmField var depositError = 6.0
        @JvmField var intakeCrossingVelo = 26.0
        @JvmField var intakeVelo = 26.0
        @JvmField var intakeAngle = 1.0
        @JvmField var depositVelo = 60.0
        @JvmField var angleOffset = -11.0
        @JvmField var initialOffset = 15.0
        @JvmField var yIncrement = -0.0
        @JvmField var offsetNext = true
        @JvmField var skipNext = true
        @JvmField var shouldWait = true
        @JvmField var intakeMinX = 48.0
        @JvmField var pathRotationOffset = -0.5
    }

    enum class PathState {
        INTAKING,
        DUMPING,
        IDLE,
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CYCLING
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.RED)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        relocalizer = robot.relocalizer
        robot.visionInit()
        val leftSequence = leftAuto()
        val middleSequence = middleAuto()
        val rightSequence = rightAuto()
        val parkEmergency = robot.trajectorySequenceBuilder(Pose2d(allianceHub.center.polarAdd(
            cyclingDistance, Math.toRadians(cyclingAngle).flip(blue)), Math.toRadians(cyclingAngle).flip(blue)))
            .splineTo(Vector2d(20.0, coast - 5).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(stop, coast - 5).flip(blue), 0.0)
            .build()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        deposit.liftUp()
        waitForStart()
        Async.start {
            while (opModeIsActive()) {
                if (robot.pathState == PathState.INTAKING) {
                    try {
                        robot.relocalizer.updatePoseEstimate(
                            Relocalizer.Sensor.FRONT_RIGHT,
                            null
                        )
                    } catch (e: Exception) {

                    }
                }
            }
        }
        val timer = ElapsedTime()
        timer.reset()
        robot.scan()
        val sequence = when (location) {
            TSEDetector.Location.LEFT -> leftSequence
            TSEDetector.Location.MIDDLE -> middleSequence
            TSEDetector.Location.RIGHT -> rightSequence
        }
        deposit.setLevel(Robot.getLevel(location))
        robot.turnOffVision()
        deposit.toggleCloseDeposit()
        robot.followTrajectorySequenceAsync(sequence)
        var incremented = false
        while (robot.isBusy && stopCondition()) {
            robot.update()
            when (robot.pathState) {
                PathState.INTAKING -> {
                    admissibleError = Pose2d(intakeError, intakeError, Math.toRadians(20.0))
                    Robot.admissibleTimeout = 0.2
                    Robot.gainMode = if (robot.isOverPoles) GainMode.FORWARD else GainMode.IDLE
                    Robot.admissibleVelo = Pose2d(15.0, 15.0, Math.toRadians(60.0))
                    if (robot.poseEstimate.x > intakeMinX && intake.containsBlock && !incremented && skipNext) {
                        robot.nextSegment(offsetNext)
                        incremented = true
                    }
                }
                PathState.DUMPING -> {
                    incremented = false
                    admissibleError = Pose2d(depositError, depositError, Math.toRadians(30.0))
                    Robot.admissibleTimeout = depositingTimeout
                    Robot.gainMode = if (robot.isOverPoles) GainMode.BACKWARD else GainMode.IDLE
                    Robot.admissibleVelo = Pose2d(15.0, 15.0, Math.toRadians(60.0))
//                    if (timer.seconds() >= 29) {
//                        deposit.liftDown()
//                        robot.followTrajectorySequenceAsync(parkEmergency)
//                        while (robot.isBusy && opModeIsActive()) {
//                            robot.update()
//                            Robot.gainMode = if (robot.isOverPoles) GainMode.FORWARD else GainMode.IDLE
//                        }
//                        break
//                    }
                }
                else -> {
                    Robot.gainMode = GainMode.IDLE
                }
            }
        }
    }
    fun randomRange(lower: Double, upper: Double): Double{
        return Math.random() * upper + Math.random() * lower
    }
    private fun signalTurn(t: Double): DriveSignal {
        if (robot.poseEstimate.x < 50) return DriveSignal(Pose2d(30.0))
        return DriveSignal(Pose2d(Range.clip(50.0*sin(2*t), -5.0, 15.0), 0.0))
    }
    private fun waitingFor() : Boolean {
        return if (shouldWait) deposit.isTransitioningState()
        else false
    }
    private fun theRest(trajectoryBuilder: TrajectorySequenceBuilder<PathState>): TrajectorySequence {
        var coast = coast
        for (i in 1..cycles) {
            val angle = Math.toRadians(randomRange(-intakeAngle, 0.0)).flip(blue)
            val forwardTangent = Math.toRadians(pathRotationOffset).flip(blue)
            val conjoiningVec = Vector2d(conjoiningPoint, coast).flip(blue)
            trajectoryBuilder
                .setState(PathState.INTAKING)
                .splineTo(conjoiningVec, forwardTangent)
                .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                    intake.setPower(0.1)
                    deposit.liftDown()
                    //deposit.toggleMediumDeposit()
                }
                .UNSTABLE_addDisplacementMarkerOffset(powerDelay) {
                    intake.setPower(1.0)
                    deposit.liftDown()
                }
                .liftLevel(deposit, Deposit.Level.LEVEL3)
                .increaseGains(intakeCrossingVelo)
                .splineToConstantHeading(conjoiningVec.polarAdd(gainsPoint - conjoiningPoint, forwardTangent), forwardTangent)
                .increaseGains(intakeVelo)
                .splineTo(conjoiningVec
                    .polarAdd(gainsPoint - conjoiningPoint, forwardTangent)
                    .polarAdd(stop - gainsPoint + i / divConstant, forwardTangent + angle)
                    ,forwardTangent + angle)
                .defaultGains()
                .waitSeconds(waitTime)
                .relocalize(robot)
                .setReversed(true)
                .intakeOff(intake)
            trajectoryBuilder
                .setState(PathState.DUMPING)
                .increaseGains(depositVelo)
                .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), Math.PI + forwardTangent) // change
                .UNSTABLE_addDisplacementMarkerOffset(depositDelay, deposit::liftUp)
                .splineTo(
                    allianceHub.center.polarAdd(
                        cyclingDistance,
                        Math.toRadians(cyclingAngle).flip(blue)
                    ),
                    allianceHub.center,
                    Pose2d(0.0, 0.0, Math.toRadians(angleOffset).flip(blue))
                )
                .defaultGains()
                .waitWhile(this::waitingFor)
                .hardDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
            coast -= yIncrement
        }
        return trajectoryBuilder
            .setState(PathState.IDLE)
            .setVelConstraint(Robot.getVelocityConstraint(70.0, Math.toRadians(200.0), DriveConstants.TRACK_WIDTH))
            .setAccelConstraint(Robot.getAccelerationConstraint(150.0))
            .splineTo(Vector2d(20.0, coast).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(stop + 10, coast).flip(blue), 0.0)
            .build()
    }
    private fun stopCondition(): Boolean {
        if (robot.pathState == PathState.IDLE) return true
        return opModeIsActive()
    }
    private fun leftAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .liftLevel(deposit, Deposit.Level.LEVEL1)
                .splineTo(
                    allianceHub.center.polarAdd(
                        depositDistance,
                        Math.toRadians(depositingAngle).flip(blue)),
                    allianceHub.center,
                    Pose2d(0.0, 0.0, Math.toRadians(initialOffset).flip(blue)),
                )
                .setReversed(false)
                .waitWhile(deposit::isTransitioningState)
                .hardDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
    private fun middleAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .liftLevel(deposit, Deposit.Level.LEVEL2)
                .splineTo(
                    allianceHub.center.polarAdd(
                        depositDistance,
                        Math.toRadians(depositingAngle).flip(blue)),
                    allianceHub.center,
                    Pose2d(0.0, 0.0, Math.toRadians(initialOffset).flip(blue)),
                )
                .setReversed(false)
                .waitWhile(deposit::isTransitioningState)
                .hardDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition/ wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
    private fun rightAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .liftLevel(deposit, Deposit.Level.LEVEL3)
                .splineTo(
                    allianceHub.center.polarAdd(
                        depositDistance,
                        Math.toRadians(depositingAngle).flip(blue)),
                    allianceHub.center,
                    Pose2d(0.0, 0.0, Math.toRadians(initialOffset).flip(blue)),
                )
                .setReversed(false)
                .waitWhile(deposit::isTransitioningState)
                .hardDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
}