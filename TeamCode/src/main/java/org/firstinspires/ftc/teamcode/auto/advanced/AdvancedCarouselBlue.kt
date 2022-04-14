package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.allianceHub
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.carouselVec
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context.location
import org.firstinspires.ftc.teamcode.util.field.Context.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import kotlin.Throws

@Autonomous
@Disabled
class AdvancedCarouselBlue : LinearOpMode() {
    lateinit var robot: Robot<PathState>
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    private val blue = true

    enum class PathState {
        INTAKING,
        DUMPING,
        IDLE,
    }

    companion object {
        @JvmField var coast = -57.2
        @JvmField var stop = 52.0
        @JvmField var intakeDelay = 16.5
        @JvmField var depositDelay = 18.0
        @JvmField var closeDist = 25.0
        @JvmField var conjoiningPoint = 30.0
        @JvmField var conjoiningDeposit = 30.0
        @JvmField var waitTime = 0.1
        @JvmField var gainsPoint = 36.0
        @JvmField var cyclingDistance = 26.0
        @JvmField var depositDistance = 29.5
        @JvmField var divConstant = 2.0
        @JvmField var depositingAngle = -60.0
        @JvmField var cyclingAngle = -55.0
        @JvmField var depositingTimeout = 0.4
        @JvmField var intakeError = 8.0
        @JvmField var depositError = 8.0
        @JvmField var intakeCrossingVelo = 30.0
        @JvmField var intakeVelo = 50.0
        @JvmField var intakeAngle = 5.0
        @JvmField var depositVelo = 60.0
        @JvmField var angleOffset = -12.0
        @JvmField var yIncrement = -0.05
        @JvmField
        var polesCoast = -35.0
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CAROUSEL
        robot = Robot(this, startingPosition(), OpModeType.AUTO, Alliance.BLUE)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        robot.visionInit()
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
                .liftLevel(deposit, Robot.getLevel(location))
                .splineTo(
                    allianceHub.center.polarAdd(
                        cyclingDistance, Math.toRadians(
                            -125.0
                        ).flip(blue)
                    ), allianceHub.center
                )
                .setReversed(false)
                .softDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                .splineTo(
                    carouselVec.center.polarAdd(
                        carouselVec.radius + 3.0,
                        Math.toRadians(40.0).flip(blue)
                    ), carouselVec.center,
                    Pose2d(0.0, 0.0, Math.toRadians(20.0))
                )
                .waitSeconds(2.0, DriveSignal(Pose2d(5.0)))
                .carouselOff(carousel) // drop the ducky
                //.intakeOn(intake)
                .back(5.0)
                .turn(Math.toRadians(60.0).flip(blue))
                .turn(Math.toRadians(-120.0).flip(blue))
                .intakeOff(intake)
                .setReversed(true)
                .liftLevel(deposit, Deposit.Level.LEVEL3)
                .splineTo(
                    allianceHub.center.polarAdd(
                        cyclingDistance, Math.toRadians(
                            -125.0
                        ).flip(blue)
                    ), allianceHub.center,
                )
                .softDump(deposit)
                .waitWhile(deposit::isDoingWork)
                .setReversed(false)
                .splineTo(Vector2d(-55.0, -46.0).flip(blue), Math.toRadians(90.0).flip(blue))
                .splineTo(Vector2d(-55.0, -27.0).flip(blue), Math.toRadians(90.0).flip(blue))
                .splineTo(Vector2d(-12.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
        for (i in 1..2)
            trajectoryBuilder
                .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                    //intake.setPower(1.0)
                }
                .splineTo(Vector2d(16.0, -20.0).flip(blue), Math.toRadians(-40.0).flip(blue))
                .increaseGains(30.0)
                .splineToConstantHeading(Vector2d(16.0, -20.0).flip(blue).polarAdd(20.0, Math.toRadians(-40.0).flip(blue)), Math.toRadians(-40.0).flip(blue))
                .defaultGains()
                .splineTo(
                    Vector2d(50.0, -45.0).flip(blue),
                    Math.toRadians(-30.0 - 20 * Math.random()).flip(blue)
                )
                .turn(Math.toRadians(30.0).flip(blue))
                .setReversed(true)
                .intakeOff(intake)
                .splineTo(Vector2d(16.0, -20.0).flip(blue).polarAdd(20.0, Math.toRadians(-40.0).flip(blue)), Math.toRadians(180.0 - 40.0).flip(blue))
                .increaseGains(30.0)
                .splineToConstantHeading(Vector2d(16.0, -20.0).flip(blue), Math.toRadians(180 - 40.0).flip(blue))
                .defaultGains()
                .liftLevel(deposit, Deposit.Level.LEVEL3)
                .splineTo(
                    allianceHub.center.polarAdd(
                        closeDist, Math.toRadians(40.0).flip(blue)
                    ), allianceHub.center
                )
                .softDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .setReversed(false)
        val sequence = trajectoryBuilder
            .splineTo(Vector2d(16.0, -20.0).flip(blue), Math.toRadians(-40.0).flip(blue))
            .splineTo(Vector2d(50.0, polesCoast).flip(blue), Math.toRadians(0.0).flip(blue))
            .build()
        robot.followTrajectorySequence(sequence)
    }
}