package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.allianceHub
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
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
import org.firstinspires.ftc.teamcode.util.field.Context
import kotlin.Throws
import kotlin.math.sin

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
        @JvmField var stop = 50.0
        @JvmField var intakeDelay = 8.5
        @JvmField var conjoiningPoint = 27.0
        @JvmField var waitTime = 0.4
        @JvmField var gainsPoint = 36.0
        @JvmField var depositDistance = 21.4
        @JvmField var divConstant = 1.4
        @JvmField var depositingAngle = -60.0
        @JvmField var intakingAngle = -20.0
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
//        robot.visionInit()
        val leftSequence = leftAuto()
        val middleSequence = middleAuto()
        val rightSequence = rightAuto()
        while (!opModeIsActive() && !isStopRequested) {
            // robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        waitForStart()
        // robot.scan()
        val sequence = when (location) {
            TSEDetector.Location.LEFT -> leftSequence
            TSEDetector.Location.MIDDLE -> middleSequence
            TSEDetector.Location.RIGHT -> rightSequence
        }
        // robot.turnOffVision()
        robot.followTrajectorySequenceAsync(sequence)
        while (robot.isBusy && opModeIsActive()) {
            Context.packet.put("Path State", robot.pathState)
            robot.update()
            when (robot.pathState) {
                PathState.INTAKING -> {
                    admissibleError = Pose2d(10.0, 10.0, Math.toRadians(20.0))
                    Robot.admissibleTimeout = 0.5
                }
                PathState.DUMPING -> {
                    admissibleError = Pose2d(2.0, 2.0, Math.toRadians(8.0))
                    Robot.admissibleTimeout = 0.2
                }
                else -> {

                }
            }
        }
    }
    private fun signalTurn(t: Double): DriveSignal {
        return DriveSignal(Pose2d(20.0, 0.0, -sin(3*t)))
    }
    private fun theRest(trajectoryBuilder: TrajectorySequenceBuilder<PathState>): TrajectorySequence {
        for (i in 1..6) {
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                    intake.setPower(1.0)
                }
                .setState(PathState.INTAKING)
                .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), 0.0)
                .increaseGains(Robot.GainMode.FORWARD)
                //.carouselOn(carousel)
                .splineToConstantHeading(Vector2d(gainsPoint, coast).flip(blue), 0.0)
                .defaultGains()
                //.carouselOff(carousel)
                .splineTo(
                    Vector2d(stop + i / divConstant, intakeY - i / 2).flip(blue),
                    Math.toRadians(intakingAngle - 20 * Math.random()).flip(blue)
                )
//                .waitWhile(::signalTurn) {
//                    intake.state == Intake.State.OUT
//                }
                .waitSeconds(waitTime)
                .setReversed(true)
                .relocalize(robot)
                .intakeOff(intake)
            trajectoryBuilder
                .setState(PathState.DUMPING)
                .splineTo(Vector2d(39.0, coast).flip(blue), Math.PI)
                .splineToConstantHeading(Vector2d(gainsPoint, coast).flip(blue), Math.PI)
                .increaseGains(Robot.GainMode.BACKWARD)
                .splineToConstantHeading(Vector2d(conjoiningPoint, coast).flip(blue), Math.PI)
                .defaultGains()
                //.liftUp(deposit, Deposit.State.LEVEL3)
                .splineTo(
                    allianceHub.center.polarAdd(
                        depositDistance,
                        Math.toRadians(depositingAngle).flip(blue)
                    ),
                    allianceHub.center
                )
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        }
        return trajectoryBuilder
            .splineTo(Vector2d(20.0, coast).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(stop, coast).flip(blue), 0.0)
            .build()
    }
    private fun leftAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .splineTo(allianceHub.center.polarAdd(depositDistance, Math.toRadians(
                    depositingAngle).flip(blue)), allianceHub.center)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
    private fun middleAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
//                        .capstoneReady(capstone)
//                        .splineToVectorOffset(barcode[1].vec().flip(blue), Pose2d(14.0, -8.0), (Math.PI / 2 + barcode[1].heading).flip(blue))
//                        .capstonePickup(capstone)
//                        .liftUp(deposit, Robot.getLevel(location))
//                        .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(allianceHub.center.polarAdd(depositDistance, Math.toRadians(-60.0).flip(blue)), allianceHub.center)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition/ wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
    private fun rightAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
//                        .capstoneReady(capstone)
//                        .splineToVectorOffset(barcode[1].vec().flip(blue), Pose2d(14.0, -8.0), (Math.PI / 2 + barcode[1].heading).flip(blue))
//                        .capstonePickup(capstone)
//                        .liftUp(deposit, Robot.getLevel(location))
//                        .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(allianceHub.center.polarAdd(allianceHub.radius, Math.toRadians(-60.0).flip(blue)), allianceHub.center)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
}