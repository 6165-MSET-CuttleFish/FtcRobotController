package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
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
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.Line
import kotlin.Throws

@Autonomous
@Config
class CyclingBlue : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    lateinit var relocalizer: Relocalizer
    private val blue = true
    companion object {
        @JvmField var radiusOffset = 5.0
        @JvmField var line = -44.0
        @JvmField var coast = -55.0
        @JvmField var stop = 47.0
        @JvmField var intakeDelay = 8.0
        @JvmField var conjoiningPoint = 18.0
        @JvmField var waitTime = 1.0
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CYCLING
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.BLUE)
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
        robot.followTrajectorySequence(sequence)
    }
    private fun theRest(trajectoryBuilder: TrajectorySequenceBuilder): TrajectorySequence {
        for (i in 1..9)
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                    intake.setPower(1.0)
                }
                .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), 0.0)
                .increaseGains(Robot.GainMode.FORWARD)
                .splineToConstantHeading(Vector2d(28.0, coast).flip(blue), 0.0)
                .defaultGains()
                .splineTo(Vector2d(stop, coast).flip(blue), Math.toRadians(0.0 - 20 * Math.random()).flip(blue))
                .setReversed(true)
                .waitSeconds(waitTime)
                .relocalize(robot)
                .intakeOff(intake)
                .splineTo(Vector2d(39.0, coast).flip(blue), Math.PI)
                .splineToConstantHeading(Vector2d(28.0, coast).flip(blue), Math.PI)
                .increaseGains(Robot.GainMode.BACKWARD)
                .splineToConstantHeading(Vector2d(conjoiningPoint, coast).flip(blue), Math.PI)
                .defaultGains()
                .liftUp(deposit, Deposit.State.LEVEL3)
                .splineToCircle(
                    allianceHub.expandedRadius(radiusOffset),
                    Line.yAxis(line).flip(blue),
                    Vector2d(12.0, -24.0).flip(blue),
                    Pose2d(0.0, 0.0, Math.toRadians(0.0)).flip(blue)
                )
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        return trajectoryBuilder
            .splineTo(Vector2d(20.0, -55.0).flip(blue), 0.0)
            .splineToConstantHeading(Vector2d(39.0, -55.0).flip(blue), 0.0)
            .build()
    }
    private fun leftAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .splineToCircle(
                    allianceHub.expandedRadius(radiusOffset),
                    Line.yAxis(line).flip(blue),
                    Vector2d(12.0, -24.0).flip(blue),
                    Pose2d(0.0, 0.0, Math.toRadians(0.0)).flip(blue)
                )
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        return theRest(trajectoryBuilder)
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
        return theRest(trajectoryBuilder)
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
        return theRest(trajectoryBuilder)
    }
}