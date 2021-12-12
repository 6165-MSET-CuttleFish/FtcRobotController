package org.firstinspires.ftc.teamcode.auto.basic

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.DriveConstants.admissibleError
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
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
        side = Side.CAROUSEL
        robot.autoInit()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Location", location)
            telemetry.update()
        }
        waitForStart()
        robot.scan()
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .back(8.0)
                .setConstraints(getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(40.0))
                .turn(Math.toRadians(90.0).flip(blue))
                .splineTo(Vector2d(-55.0, -24.8).flip(blue), Math.toRadians(-270.0).flip(blue))
                .setConstraints(getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(10.0))
                .turn(Math.toRadians(-90.0).flip(blue))
                .forward(3.0)
                .capstoneReady(capstone)
                .waitWhile(capstone::isDoingWork)
                .back(20.0)
                .resetConstraints()
                .capstonePickup(capstone)
                .liftUp(deposit, getLevel(location))
                .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(Vector2d(-27.9, -25.0).flip(blue), Math.toRadians(30.0).flip(blue))
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dump
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    admissibleError = Pose2d(5.0, 5.0, Math.toRadians(20.0))
                }
                .setVelConstraint(getVelocityConstraint(15.0, Math.PI,15.0))
                // .splineTo(Vector2d(-45.5, -45.5).flip(blue), Math.toRadians(215.0).flip(blue))
                // .setVelConstraint(getVelocityConstraint(5.0, Math.PI,15.0))
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    admissibleError = Pose2d(2.0, 2.0, Math.toRadians(5.0))
                }
                .splineTo(Vector2d(-58.0, -53.0).flip(blue), Math.toRadians(203.0).flip(blue))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, carousel::on)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, carousel::on)
                .waitSeconds(3.0)
                .carouselOff(carousel)// drop the ducky
                .resetConstraints()
                .setReversed(true)
                .splineTo(Vector2d(-61.0, -34.0).flip(blue), Math.toRadians(180.0).flip(blue))
        val trajectorySequence = trajectoryBuilder
            .build()
        robot.turnOffVision()
        robot.followTrajectorySequence(trajectorySequence)
    }
}