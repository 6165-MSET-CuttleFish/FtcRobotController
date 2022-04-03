package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.*
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
import org.firstinspires.ftc.teamcode.roadrunnerext.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.Async
import org.firstinspires.ftc.teamcode.util.field.Context
import kotlin.Throws

@Autonomous
@Config
class CyclingRed : LinearOpMode() {
    lateinit var robot: Robot<PathState>
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    lateinit var relocalizer: Relocalizer
    private val blue = false
    companion object {
        @JvmField var coast = -55.0
        @JvmField var intakeY = -55.0
        @JvmField var stop = 50.8
        @JvmField var intakeDelay = 16.0
        @JvmField var depositDelay = 16.0
        @JvmField var closeDist = 21.6
        @JvmField var depositWaitTime = 0.05
        @JvmField var conjoiningPoint = 14.0
        @JvmField var conjoiningDeposit = 30.0
        @JvmField var waitTime = 0.1
        @JvmField var gainsPoint = 36.0
        @JvmField var depositDistance = 24.0
        @JvmField var cyclingDistance = 22.2
        @JvmField var divConstant = 9.0
        @JvmField var depositingAngle = -60.0
        @JvmField var cyclingAngle = -60.0
        @JvmField var intakingAngle = 0.0
        @JvmField var depositingTimeout = 0.3
        @JvmField var intakeError = 10.0
        @JvmField var intakeVelo = 26.0
        @JvmField var depositVelo = 60.0
        @JvmField var angleOffset = -4.0
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
            .splineTo(Vector2d(20.0, coast).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(stop, coast).flip(blue), 0.0)
            .build()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        Deposit.allowLift = true
        waitForStart()
        Async.start {
            while (opModeIsActive()) {
                if (robot.pathState == PathState.INTAKING) {
                    try {
                        robot.relocalizer.updatePoseEstimate(
                            Relocalizer.Sensor.FRONT_LEFT,
                            Relocalizer.Sensor.RIGHT
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
        robot.followTrajectorySequenceAsync(sequence)
        while (robot.isBusy && opModeIsActive()) {
            Context.packet.put("Path State", robot.pathState)
            robot.update()
            when (robot.pathState) {
                PathState.INTAKING -> {
                    admissibleError = Pose2d(intakeError, intakeError, Math.toRadians(40.0))
                    Robot.admissibleTimeout = 0.1
                    if (robot.isOverPoles) {
                        //carousel.setPower(1.0)
                        Robot.gainMode = GainMode.FORWARD
                    } else {
                        //carousel.setPower(0.0)
                        Robot.gainMode = GainMode.IDLE
                    }
                }
                PathState.DUMPING -> {
                    admissibleError = Pose2d(2.0, 2.0, Math.toRadians(10.0))
                    Robot.admissibleTimeout = depositingTimeout
                    if (robot.isOverPoles) {
                        //carousel.setPower(1.0)
                        Robot.gainMode = GainMode.BACKWARD
                    } else {
                        //carousel.setPower(0.0)
                        Robot.gainMode = GainMode.IDLE
                    }
                    if (timer.seconds() > 29) {
                        Deposit.allowLift = false
                        robot.followTrajectorySequenceAsync(parkEmergency)
                        while (robot.isBusy && opModeIsActive()) {
                            robot.update()
                            if (robot.isOverPoles) {
                                //carousel.setPower(1.0)
                                Robot.gainMode = GainMode.FORWARD
                            } else {
                                //carousel.setPower(0.0)
                                Robot.gainMode = GainMode.IDLE
                            }
                        }
                        break
                    }
                }
                else -> {
                    Robot.gainMode = GainMode.IDLE
                }
            }
        }
    }
    private fun signalTurn(t: Double): DriveSignal {
        return DriveSignal(Pose2d(20.0, 0.0, Math.toRadians(-70.0)))
    }
    private fun theRest(trajectoryBuilder: TrajectorySequenceBuilder<PathState>): TrajectorySequence {
        for (i in 1..5) {
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                    intake.setPower(1.0)
                }
                .setState(PathState.INTAKING)
                .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), 0.0)
                .increaseGains(intakeVelo)
                //.carouselOn(carousel)
                .splineToConstantHeading(Vector2d(stop, coast).flip(blue), 0.0)
                .defaultGains()
                //.carouselOff(carousel)
                .splineToConstantHeading(Vector2d(stop + i / divConstant, coast).flip(blue),0.0)
//                .waitWhile(::signalTurn) {
//                    intake.state == Intake.State.OUT
//                }
                .waitSeconds(waitTime)
                .relocalize(robot)
                .setReversed(true)
                .intakeOff(intake)
                .liftUp(deposit, Deposit.Level.LEVEL3)
            trajectoryBuilder
                .setState(PathState.DUMPING)
                .splineTo(Vector2d(39.0, coast).flip(blue), Math.PI)
                .splineToConstantHeading(Vector2d(gainsPoint, coast).flip(blue), Math.PI)
                .UNSTABLE_addDisplacementMarkerOffset(depositDelay) {
                    Deposit.allowLift = true
                }
                .increaseGains(depositVelo)
                .splineToConstantHeading(Vector2d(conjoiningDeposit, coast).flip(blue), Math.PI)
                .defaultGains()
                .splineTo(
                    allianceHub.center.polarAdd(
                        cyclingDistance,
                        Math.toRadians(cyclingAngle).flip(blue)
                    ),
                    allianceHub.center,
                    Pose2d(0.0, 0.0, Math.toRadians(angleOffset).flip(blue))
                )
                .waitWhile(deposit::isTransitioningState)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        }
        return trajectoryBuilder
            .setState(PathState.IDLE)
            .splineTo(Vector2d(20.0, coast).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(stop, coast).flip(blue), 0.0)
            .build()
    }
    private fun leftAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .waitSeconds(0.4)
                .liftUp(deposit, Deposit.Level.LEVEL1)
                .splineTo(allianceHub.center.polarAdd(
                    closeDist, Math.toRadians(
                        depositingAngle).flip(blue)), allianceHub.center)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
    private fun middleAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .liftUp(deposit, Deposit.Level.LEVEL2)
                .waitSeconds(0.4)
                .splineTo(allianceHub.center.polarAdd(closeDist, Math.toRadians(-60.0).flip(blue)), allianceHub.center)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition/ wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
    private fun rightAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .liftUp(deposit, Deposit.Level.LEVEL3)
                .splineTo(allianceHub.center.polarAdd(depositDistance, Math.toRadians(-60.0).flip(blue)), allianceHub.center)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
}