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
import org.firstinspires.ftc.teamcode.modules.vision.TSEDetector
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context.location
import org.firstinspires.ftc.teamcode.util.field.Context.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
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
        @JvmField var cyclingDistance = 22.0
        @JvmField var carouselAngle = 50.0
        @JvmField var carouselDistance = 25.0
        @JvmField var carouselAngleOffset = 40.0
        @JvmField var cyclingAngle = -130.0
        @JvmField var vel = 35.0
        @JvmField var accel = 40.0
        @JvmField var carouselCoast = -52.0
        @JvmField var forwardDist = 13.0
        @JvmField var carouselPower = 0.2
        @JvmField var carouselTurn = 0.0
        @JvmField var carouselForward = 3.0
        @JvmField var waitTime = 4.0
        @JvmField var carouselMovingSpeed = 15.0
        @JvmField var parkY = -40.0
        @JvmField var angleOffset = 10.0
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
        val leftSequence = leftAuto()
        val middleSequence = middleAuto()
        val rightSequence = rightAuto()
        deposit.liftUp()
        deposit.toggleCloseDeposit()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Position", location)
            telemetry.update()
        }
        waitForStart()
        robot.scan()
        val sequence = when (location) {
            TSEDetector.Location.LEFT -> leftSequence
            TSEDetector.Location.MIDDLE -> middleSequence
            TSEDetector.Location.RIGHT -> rightSequence
        }
        robot.turnOffVision()
        robot.followTrajectorySequence(sequence)
    }

    private fun leftAuto(): TrajectorySequence {
        val builder = robot.trajectorySequenceBuilder(startingPosition())
            .setAccelConstraint(Robot.getAccelerationConstraint(accel))
            .setVelConstraint(Robot.getVelocityConstraint(vel, Math.toRadians(200.0), Math.toRadians(200.0)))
            .setReversed(true)
            .liftLevel(deposit, Deposit.Level.LEVEL1)
        return theRest(builder)
    }

    private fun middleAuto(): TrajectorySequence {
        val builder = robot.trajectorySequenceBuilder(startingPosition())
            .setAccelConstraint(Robot.getAccelerationConstraint(accel))
            .setVelConstraint(Robot.getVelocityConstraint(vel, Math.toRadians(200.0), Math.toRadians(200.0)))
            .setReversed(true)
            .liftLevel(deposit, Deposit.Level.LEVEL2)
        return theRest(builder)
    }

    private fun rightAuto(): TrajectorySequence {
        val builder = robot.trajectorySequenceBuilder(startingPosition())
            .setAccelConstraint(Robot.getAccelerationConstraint(accel))
            .setVelConstraint(Robot.getVelocityConstraint(vel, Math.toRadians(200.0), Math.toRadians(200.0)))
            .setReversed(true)
            .liftLevel(deposit, Deposit.Level.LEVEL3)
        return theRest(builder)
    }

    private fun theRest(trajectorySequenceBuilder: TrajectorySequenceBuilder<*>): TrajectorySequence {
        return trajectorySequenceBuilder.splineTo(
            allianceHub.center.polarAdd(
                cyclingDistance, Math.toRadians(
                    cyclingAngle
                ).flip(blue)
            ), allianceHub.center
        )
            .setReversed(false)
            .softDump(deposit)
            .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
            .UNSTABLE_addDisplacementMarkerOffset(1.0) {
                carousel.setPower(carouselPower)
            }
            .splineTo(Vector2d(-35.0, carouselCoast).flip(blue), Math.PI)
            .increaseGains(carouselMovingSpeed)
            .splineTo(
                carouselVec.center.polarAdd(
                    carouselDistance,
                    Math.toRadians(carouselAngle).flip(blue)
                ), carouselVec.center,
                Pose2d(0.0, 0.0, Math.toRadians(carouselAngleOffset))
            )
            .setAccelConstraint(Robot.getAccelerationConstraint(accel))
            .setVelConstraint(Robot.getVelocityConstraint(vel, Math.toRadians(200.0), Math.toRadians(200.0)))
            .waitSeconds(waitTime, DriveSignal(Pose2d(carouselForward, 0.0, Math.toRadians(-carouselTurn))))
            .carouselOff(carousel) // drop the ducky
            .setReversed(true)
            .back(5.0)
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                intake.stepbro(Intake.inPosition)
            }
            .splineTo(
                carouselVec.center.polarAdd(
                    carouselDistance + 5,
                    Math.toRadians(carouselAngle).flip(blue)
                ), carouselVec.center,
                Pose2d(0.0, 0.0, Math.PI)
            )
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                intake.stepbro(0.35)
            }
            .setTurnConstraint(Math.toRadians(120.0), Math.toRadians(150.0))
            .turn(Math.toRadians(90.0).flip(blue))
            .turn(Math.toRadians(-150.0).flip(blue))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                intake.stepbro(0.45)
            }
            .turn(Math.toRadians(150.0).flip(blue))
            .turn(Math.toRadians(-150.0).flip(blue))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                intake.stepsis()
            }
            .waitSeconds(0.1)
            .setReversed(true)
            .liftLevel(deposit, Deposit.Level.LEVEL2)
            .splineTo(
                allianceHub.center.polarAdd(
                    cyclingDistance, Math.toRadians(
                        cyclingAngle
                    ).flip(blue)
                ), allianceHub.center,
                Pose2d(0.0, 0.0, Math.toRadians(angleOffset).flip(blue))
            )
            .performAction {
                deposit.liftUp()
            }
            .waitSeconds(0.3)
            .waitWhile(deposit::isTransitioningState)
            .hardDump(deposit)
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                intake.dontFlipOut = true
            }
            .waitWhile(deposit::isDoingWork)
            .setReversed(false)
            .splineTo(Vector2d(-60.0, parkY).flip(blue), Math.toRadians(180.0).flip(blue))
            .turn(Math.PI.flip(blue) / 2)
            .back(forwardDist)
            .build()
    }
}