package org.firstinspires.ftc.teamcode.auto.advanced

import com.acmerobotics.dashboard.config.Config
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
@Config
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
        @JvmField var cyclingDistance = 25.0
        @JvmField var carouselAngle = 40.0
        @JvmField var carouselDistance = 20.0
        @JvmField var carouselAngleOffset = 20.0
        @JvmField var cyclingAngle = -125.0
        @JvmField var vel = 40.0
        @JvmField var accel = 50.0
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
        deposit.liftUp()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        waitForStart()
        robot.scan()
        robot.turnOffVision()
        val sequence =
            robot.trajectorySequenceBuilder(startingPosition())
                .setAccelConstraint(Robot.getAccelerationConstraint(accel))
                .setVelConstraint(Robot.getVelocityConstraint(vel, Math.toRadians(200.0), Math.toRadians(200.0)))
                .setReversed(true)
                .liftLevel(deposit, Robot.getLevel(location))
                .splineTo(
                    allianceHub.center.polarAdd(
                        cyclingDistance, Math.toRadians(
                            cyclingAngle
                        ).flip(blue)
                    ), allianceHub.center
                )
                .setReversed(false)
                .hardDump(deposit)
                .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                .UNSTABLE_addDisplacementMarkerOffset(1.0) {
                    carousel.setPower(0.3)
                }
                .splineTo(
                    carouselVec.center.polarAdd(
                        carouselDistance,
                        Math.toRadians(carouselAngle).flip(blue)
                    ), carouselVec.center,
                    Pose2d(0.0, 0.0, Math.toRadians(carouselAngleOffset))
                )
                .waitSeconds(2.0, DriveSignal(Pose2d(5.0)))
                .carouselOff(carousel) // drop the ducky
                .back(5.0)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    intake.stepbro()
                }
                .setTurnConstraint(Math.toRadians(100.0), Math.toRadians(100.0))
                .turn(Math.toRadians(60.0).flip(blue))
                .turn(Math.toRadians(-120.0).flip(blue))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    intake.stepsis()
                }
                .setReversed(true)
                .liftLevel(deposit, Deposit.Level.LEVEL3)
                .splineTo(
                    allianceHub.center.polarAdd(
                        cyclingDistance, Math.toRadians(
                            cyclingAngle
                        ).flip(blue)
                    ), allianceHub.center,
                )
                .softDump(deposit)
                .waitWhile(deposit::isDoingWork)
                .setReversed(false)
                //.splineTo(Vector2d(-55.0, -46.0).flip(blue), Math.toRadians(90.0).flip(blue))
            .build()
        robot.followTrajectorySequence(sequence)
    }
}