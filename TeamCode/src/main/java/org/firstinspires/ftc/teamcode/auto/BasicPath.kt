package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.*
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.capstone.Slides
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Details.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.util.roadrunnerext.flip
import kotlin.Throws

@Autonomous
class BasicPath : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot = Robot(this, OpModeType.AUTO, Alliance.RED)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        val blue = false
        side = Side.CAROUSEL
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .capstoneReady(capstone)
                .splineTo(
                    duckLocations()[0].vec(),
                    Math.toRadians(90.0).flip(blue) + duckLocations()[0].heading
                )
                .capstonePickup(capstone)
                .liftUp(deposit)
                .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(Vector2d(-26.0, -34.0).flip(blue), Math.toRadians(30.0).flip(blue))
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dump
                .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                .setVelConstraint(getVelocityConstraint(10.0, Math.PI,15.0))
                .splineTo(Vector2d(-60.0, -54.0).flip(blue), Math.toRadians(215.0).flip(blue))
                .waitSeconds(1.5)
                .carouselOff(carousel)// drop the ducky
                .resetConstraints()
                .setReversed(true)
                .splineTo(Vector2d(-61.0, -35.0).flip(blue), Math.toRadians(180.0).flip(blue))
        val trajectorySequence = trajectoryBuilder
                .build()

        waitForStart()
        robot.followTrajectorySequence(trajectorySequence)
    }
}