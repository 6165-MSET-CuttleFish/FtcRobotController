package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.duckLocation
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.dumpPosition
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Details.location
import org.firstinspires.ftc.teamcode.util.field.Details.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.roadrunnerext.flip
import kotlin.Throws

class AdvancedCarouselRed : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    private val blue = true

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CYCLING
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.BLUE)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        robot.autoInit()
//        val leftSequence = leftAuto()
//        val middleSequence = middleAuto()
//        val rightSequence = rightAuto()
        while (!opModeIsActive()) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        waitForStart()
        robot.scan()
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .capstoneReady(capstone)
                .splineTo(
                    duckLocation().vec(),
                    Math.toRadians(90.0).flip(blue) + duckLocation().heading
                )
                .capstonePickup(capstone)
                .liftUp(deposit, Deposit.State.LEVEL3)
                .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(dumpPosition().vec(), dumpPosition().heading)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                .splineTo(Vector2d(-55.0, -55.0).flip(blue), Math.toRadians(210.0).flip(blue))
                .waitSeconds(1.5)
                .carouselOff(carousel)// drop the ducky
                .setReversed(true)
                .splineTo(Vector2d(-24.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                .turn(Math.toRadians(-180.0).flip(blue))
                .setReversed(false)
                .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
                .splineTo(Vector2d(10.0, -30.0).flip(blue), Math.toRadians(-90.0).flip(blue))
        for (i in 1..3)
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                    intake.setPower(1.0)
                }
                .splineTo(Vector2d(20.0, -40.0), 0.0)
                .splineTo(
                    Vector2d(39.0, -50.0).plus(
                        Vector2d(
                            5 * Math.random(),
                            5 * Math.random()
                        )
                    ).flip(blue), Math.toRadians(-35.0 + 10 * Math.random()).flip(blue)
                )
                .setReversed(true)
                .intakeOff(intake)
                .splineTo(Vector2d(20.0, -40.0), Math.toRadians(180.0).flip(blue))
                .splineTo(Vector2d(9.0, -23.0).flip(blue), Math.toRadians(180.0).flip(blue))
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
                .turn(Math.toRadians(-90.0).flip(blue))
        val sequence = trajectoryBuilder
            .splineTo(Vector2d(20.0, -40.0), 0.0)
            .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
            .build()
        robot.followTrajectorySequence(sequence)
    }
}