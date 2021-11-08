package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.duckLocations
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
        val trajectorySequence = robot
            .trajectorySequenceBuilder(Details.robotPose)
            .setReversed(true)
            .splineTo(duckLocations()[2].vec(), Math.toRadians(90.0) + duckLocations()[0].heading)
            .waitCondition { true } // duck loaded
            .splineTo(Vector2d(-28.0, -31.0), Math.toRadians(20.0))
            .setReversed(false)
            .splineTo(Vector2d(-55.0, -55.0), Math.toRadians(210.0))
            .waitSeconds(1.5) // drop the ducky
            .setReversed(true)
            .splineTo(Vector2d(-24.0, -4.0), Math.toRadians(0.0))
            .turn(Math.toRadians(-180.0))
            .setReversed(false)
            .splineTo(Vector2d(6.0, -15.0), Math.toRadians(-90.0))
            .splineTo(Vector2d(45.0, -45.0), Math.toRadians(-30.0))
            .build()
        waitForStart()
        robot.followTrajectorySequence(trajectorySequence)
    }
}