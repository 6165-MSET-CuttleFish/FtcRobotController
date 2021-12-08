package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.*
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Details.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.util.flip
import kotlin.Throws

@Autonomous
class CyclingRed : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val blue = false
        side = Side.CYCLING
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.RED)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
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
                .splineTo(dumpPosition().vec(), Math.PI.flip(blue) + dumpPosition().heading)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        for (i in 1..6)
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                    intake.setPower(1.0)
                }
                .splineTo(
                    Vector2d(20.0, -40.0).flip(blue), 0.0
                )
                .decreaseGains()
                .splineTo(
                    Vector2d(24.0, -40.0).flip(blue), 0.0
                )
                .defaultGains()
                .splineTo(
                    Vector2d(52.0, -50.0).plus(
                        Vector2d(
                            5 * Math.random(),
                            5 * Math.random()
                        )
                    ).flip(blue), Math.toRadians(-35.0 + 10 * Math.random()).flip(blue)
                )
                .setReversed(true)
                .intakeOff(intake)
                .splineTo(
                    Vector2d(26.0, -40.0).flip(blue), Math.PI
                )
                .decreaseGains()
                .splineTo(
                    Vector2d(24.0, -40.0).flip(blue), Math.PI
                )
                .defaultGains()
                .splineTo(cycleDumpPosition().vec(), cycleDumpPosition().heading + Math.PI.flip(blue))
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        val trajectorySequence = trajectoryBuilder
            .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
            .build()
        waitForStart()
        robot.followTrajectorySequence(trajectorySequence)
    }
}