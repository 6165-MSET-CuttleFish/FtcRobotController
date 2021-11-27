package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.flip
import kotlin.Throws

@Autonomous
class CarouselRed : LinearOpMode() {
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
                .waitCondition { !capstone.isDoingWork } // capstone loaded
                .splineTo(cycleDump().vec(), cycleDump().heading)
                .setReversed(false)
                .dump(deposit)
                .waitCondition { !deposit.isDoingWork } // wait for platform to dump
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
                .splineTo(Vector2d(20.0,-40.0), 0.0)
                .splineTo(
                    Vector2d(39.0,-50.0).plus(
                        Vector2d(
                            5 * Math.random(),
                            5 * Math.random(),
                        )
                    ).flip(blue), Math.toRadians(-35.0 + 10 * Math.random()).flip(blue)
                )
                .setReversed(true)
                .intakeOff(intake)
                .splineTo(Vector2d(20.0,-40.0), Math.toRadians(180.0).flip(blue))
                .splineTo(Vector2d(9.0,-23.0).flip(blue), Math.toRadians(180.0).flip(blue))
                .dump(deposit)
                .waitCondition { !deposit.isDoingWork } // wait for platform to dump
                .setReversed(false)
                .turn(Math.toRadians(-90.0).flip(blue))
        val trajectorySequence = trajectoryBuilder
            .splineTo(Vector2d(20.0, -40.0), 0.0)
            .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
            .build()
        waitForStart()
        robot.followTrajectorySequence(trajectorySequence)
    }
}