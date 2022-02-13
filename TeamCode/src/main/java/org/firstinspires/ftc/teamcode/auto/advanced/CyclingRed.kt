package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.allianceHub
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.vision.TSEDetector
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context.location
import org.firstinspires.ftc.teamcode.util.field.Context.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.roadrunnerext.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.Line
import kotlin.Throws

@Autonomous
class CyclingRed : LinearOpMode() {
    lateinit var robot: Robot<PathState>
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    private val blue = false

    enum class PathState {
        INTAKING,
        DUMPING,
        IDLE,
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CYCLING
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.BLUE)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        robot.visionInit()
        val leftSequence = leftAuto()
        val middleSequence = middleAuto()
        val rightSequence = rightAuto()
        while (!opModeIsActive()) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        waitForStart()
        robot.scan()
        val sequence = when (location) {
            TSEDetector.Location.LEFT -> leftSequence
            TSEDetector.Location.MIDDLE -> middleSequence
            TSEDetector.Location.RIGHT -> rightSequence
        }
        robot.turnOffVision()
        robot.followTrajectorySequence(sequence)
    }
    private fun theRest(trajectoryBuilder: TrajectorySequenceBuilder<PathState>): TrajectorySequence {
        for (i in 1..9)
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                    intake.setPower(1.0)
                }
                .splineTo(Vector2d(20.0, -60.0).flip(blue), 0.0)
                .splineToConstantHeading(Vector2d(28.0, -60.0).flip(blue), 0.0)
                .splineTo(Vector2d(40.0 + i, -60.0).flip(blue), Math.toRadians(0.0 + 20 * Math.random()).flip(blue))
                .setReversed(true)
                .intakeOff(intake)
                .splineTo(Vector2d(39.0, -60.0).flip(blue), Math.PI)
                .splineToConstantHeading(Vector2d(20.0, -60.0).flip(blue), Math.PI)
                .liftUp(deposit, Deposit.State.LEVEL3)
                .splineToCircle(allianceHub.expandedRadius(9.0), Line.yAxis(-48.0).flip(blue), Vector2d(12.0, -24.0).flip(blue))
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        return trajectoryBuilder
            .splineTo(Vector2d(20.0, -60.0).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(39.0, -60.0).flip(blue), 0.0)
            .build()
    }
    private fun leftAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
//                        .capstoneReady(capstone)
//                        .splineToVectorOffset(barcode[1].vec().flip(blue), Pose2d(14.0, -8.0), (Math.PI / 2 + barcode[1].heading).flip(blue))
//                        .capstonePickup(capstone)
//                        .liftUp(deposit, Robot.getLevel(location))
//                        .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineToCircle(allianceHub.expandedRadius(9.0), Line.yAxis(-48.0).flip(blue), Vector2d(1.0, -30.0).flip(blue))
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
                .splineToCircle(allianceHub.expandedRadius(9.0), Line.yAxis(-48.0).flip(blue), Vector2d(1.0, -30.0).flip(blue))
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
                .splineToCircle(allianceHub.expandedRadius(9.0), Line.yAxis(-48.0).flip(blue), Vector2d(1.0, -30.0).flip(blue))
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        return theRest(trajectoryBuilder as TrajectorySequenceBuilder<PathState>)
    }
}