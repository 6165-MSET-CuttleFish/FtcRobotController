package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.DriveConstants.admissibleError
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.duckLocations
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.capstone.Slides
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.vision.Detector
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Details.location
import org.firstinspires.ftc.teamcode.util.field.Details.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.util.roadrunnerext.flip
import kotlin.Throws

@Autonomous
class CarouselRedNoDuck : LinearOpMode() {
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
                .setConstraints(getVelocityConstraint(30.0, DriveConstants.MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(30.0))
                .liftUp(deposit, getLevel(location))
                .splineTo(Vector2d(-11.0, -42.0).flip(blue), Math.toRadians(90.0).flip(blue))
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dump
                            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                                admissibleError = Pose2d(5.0, 5.0, Math.toRadians(20.0))
                            }
                .setVelConstraint(getVelocityConstraint(10.0, Math.PI,15.0))
                // .splineTo(Vector2d(-45.5, -45.5).flip(blue), Math.toRadians(215.0).flip(blue))
                // .setVelConstraint(getVelocityConstraint(5.0, Math.PI,15.0))
                .UNSTABLE_addTemporalMarkerOffset(0.1, carousel::on)
                            .UNSTABLE_addTemporalMarkerOffset(1.0) {
                                admissibleError = Pose2d(2.0, 2.0, Math.toRadians(5.0))
                            }
                .splineTo(Vector2d(-58.0, -53.0).flip(blue), Math.toRadians(203.0).flip(blue))
                .waitSeconds(3.0)
                .carouselOff(carousel)// drop the ducky
                .resetConstraints()
                .setReversed(true)
                .splineTo(Vector2d(-61.0, -35.0).flip(blue), Math.toRadians(180.0).flip(blue))
        val trajectorySequence = trajectoryBuilder
            .build()
        robot.turnOffVision()
        robot.followTrajectorySequence(trajectorySequence)
    }
}