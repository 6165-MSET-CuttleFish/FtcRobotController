package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.util.*
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.cycleDumpPosition
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.duckLocation
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.dumpPosition
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.getVelocityConstraint
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.vision.Detector
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Details.location
import org.firstinspires.ftc.teamcode.util.field.Details.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.util.roadrunnerext.flip
import kotlin.Throws

@Autonomous
class CyclingRed : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    private val blue = false

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CYCLING
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.RED)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        robot.autoInit()
        val leftSequence = leftAuto()
        val middleSequence = middleAuto()
        val rightSequence = rightAuto()
        while (!opModeIsActive()) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        waitForStart()
        val sequence = when (location) {
            Detector.Location.LEFT -> leftSequence
            Detector.Location.MIDDLE -> middleSequence
            Detector.Location.RIGHT -> rightSequence
        }
        robot.turnOffVision()
        robot.followTrajectorySequence(sequence)
    }
    fun leftAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .capstoneReady(capstone)
                .setVelConstraint(getVelocityConstraint(30.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(
                    duckLocation(Detector.Location.LEFT).vec(),
                    Math.toRadians(90.0).flip(blue) + duckLocation().heading
                )
                .resetConstraints()
                .capstonePickup(capstone)
                .liftUp(deposit, Robot.getLevel(Detector.Location.LEFT))
                .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(dumpPosition().vec(), Math.PI.flip(blue) + dumpPosition().heading)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        for (i in 1..1)
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
                    ).flip(blue), Math.toRadians(-35.0 + 5 * Math.random()).flip(blue)
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
        return trajectoryBuilder
            .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
            .build()
    }
    fun middleAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .capstoneReady(capstone)
                .setVelConstraint(getVelocityConstraint(30.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(
                    duckLocation(Detector.Location.MIDDLE).vec(),
                    Math.toRadians(90.0).flip(blue) + duckLocation().heading
                )
                .resetConstraints()
                .capstonePickup(capstone)
                .liftUp(deposit, Robot.getLevel(Detector.Location.MIDDLE))
                .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(dumpPosition().vec(), Math.PI.flip(blue) + dumpPosition().heading)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        for (i in 1..1)
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
                    ).flip(blue), Math.toRadians(-35.0 + 5 * Math.random()).flip(blue)
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
        return trajectoryBuilder
            .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
            .build()
    }
    fun rightAuto() : TrajectorySequence {
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition())
                .setReversed(true)
                .capstoneReady(capstone)
                .setVelConstraint(getVelocityConstraint(30.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(
                    duckLocation(Detector.Location.RIGHT).vec(),
                    Math.toRadians(90.0).flip(blue) + duckLocation().heading
                )
                .resetConstraints()
                .capstonePickup(capstone)
                .liftUp(deposit, Robot.getLevel(Detector.Location.RIGHT))
                .waitWhile(capstone::isDoingWork) // capstone loaded
                .splineTo(dumpPosition().vec(), Math.PI.flip(blue) + dumpPosition().heading)
                .setReversed(false)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
        for (i in 1..1)
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
                    ).flip(blue), Math.toRadians(-35.0 + 5 * Math.random()).flip(blue)
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
        return trajectoryBuilder
            .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
            .build()
    }
}