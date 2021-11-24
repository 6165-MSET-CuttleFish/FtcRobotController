package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
import org.firstinspires.ftc.teamcode.modules.capstone.Slides
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.Details
import org.firstinspires.ftc.teamcode.util.OpModeType
import kotlin.Throws

@Autonomous
class CarouselRed : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var slides: Slides
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot = Robot(this, OpModeType.AUTO)
        intake = robot.intake
        deposit = robot.deposit
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .splineTo(
                    duckLocations()[0].vec(),
                    Math.toRadians(90.0) + duckLocations()[0].heading
                )
                .waitCondition { true } // duck loaded
                .splineTo(cycleDump().vec(), cycleDump().heading)
                .setReversed(false)
                .waitCondition { true } // wait for platform to dump
                .splineTo(Vector2d(-55.0, -55.0), Math.toRadians(210.0))
                .waitSeconds(1.5) // drop the ducky
                .setReversed(true)
                .splineTo(Vector2d(-24.0, -4.0), Math.toRadians(0.0))
                .turn(Math.toRadians(-180.0))
                .setReversed(false)
                .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)))
                .splineTo(Vector2d(10.0, -30.0), Math.toRadians(-90.0))
        for (i in 1..5)
            trajectoryBuilder
                .splineTo(
                    Vector2d(39.0, -50.0).plus(
                        Vector2d(
                            5 * Math.random(),
                            5 * Math.random()
                        )
                    ), Math.toRadians(-35.0 + 5 * Math.random())
                )
                .setReversed(true)
                .splineTo(Vector2d(4.1, -34.0), Math.toRadians(150.0))
                .waitCondition { true } // wait for platform to dump
                .setReversed(false)
        val trajectorySequence = trajectoryBuilder
            .splineTo(Vector2d(45.0, -45.0), Math.toRadians(-35.0))
            .build()
        waitForStart()
        robot.followTrajectorySequence(trajectorySequence)
    }
}