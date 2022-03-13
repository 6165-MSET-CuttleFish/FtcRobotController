package org.firstinspires.ftc.teamcode.auto.basic


import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.allianceHub
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
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
import org.firstinspires.ftc.teamcode.roadrunnerext.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import kotlin.Throws

@Autonomous
@Config
class CarouselBlue : LinearOpMode() {
    lateinit var robot: Robot<PathState>
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    val blue = true
    companion object {
        @JvmField var carouselPower = 0.2
        @JvmField var closeDist = 20.5
    }
    enum class PathState {
        INTAKING,
        DUMPING,
        IDLE,
    }
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        side = Side.CAROUSEL
        robot = Robot(this, OpModeType.AUTO, Alliance.BLUE)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        robot.visionInit()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Location", location)
            telemetry.update()
        }
        Deposit.allowLift = false
        val left = leftAuto()
        val right = rightAuto()
        val mid = midAuto()
        waitForStart()
        robot.turnOffVision()
        val trajectorySequence = when (location) {
            TSEDetector.Location.LEFT -> left
            TSEDetector.Location.MIDDLE -> mid
            TSEDetector.Location.RIGHT -> right
        }
        robot.followTrajectorySequence(trajectorySequence)
    }

    fun theRest(builder: TrajectorySequenceBuilder<*>) : TrajectorySequenceBuilder<*> {
        return builder
            .back(7.0)
            .turn(Math.toRadians(90.0).flip(blue))
            .setReversed(true)
            .setVelConstraint(getVelocityConstraint(20.0, Math.PI,15.0))
            .splineTo(Vector2d(-51.0,-49.0).flip(blue),Math.toRadians(180.0).flip(blue))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                carousel.setPower(carouselPower)
            }
            .waitSeconds(4.0, DriveSignal(Pose2d(-5.0, 0.0, Math.toRadians(0.0))))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                carousel.setPower(-0.0)
            }
            .resetConstraints()
            .setReversed(false)
            .turn(Math.toRadians(90.0).flip(blue))
            .setVelConstraint(getVelocityConstraint(30.0, Math.PI,15.0))
            .splineTo(Vector2d(-55.0, -27.0).flip(blue), Math.toRadians(90.0).flip(blue))
            .splineTo(Vector2d(-55.0, -8.0).flip(blue), Math.toRadians(90.0).flip(blue))
            .turn(Math.toRadians(90.0).flip(blue))
            .liftUp(deposit, getLevel(location))
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                Deposit.allowLift = true
            }
            .waitSeconds(0.3)
            .setReversed(true)
    }

    fun leftAuto() : TrajectorySequence {
        val traj = theRest(robot.trajectorySequenceBuilder(startingPosition()))
            .splineTo(allianceHub.center.polarAdd(closeDist, Math.toRadians(150.0).flip(blue)), allianceHub.center)
                .dump(deposit)
                .waitWhile(deposit::isDoingWork)
                .setReversed(false)
                .splineTo(Vector2d(-58.0, -16.0).flip(blue), Math.toRadians(180.0).flip(blue))
                .turn(Math.toRadians(90.0).flip(blue))
                .splineTo(Vector2d(-58.0, -29.0).flip(blue), Math.toRadians(-90.0).flip(blue))
                .build()
        return traj
    }
    fun midAuto() : TrajectorySequence {
        val traj = theRest(robot.trajectorySequenceBuilder(startingPosition()))
            .splineTo(allianceHub.center.polarAdd(closeDist, Math.toRadians(150.0).flip(blue)), allianceHub.center)
            .dump(deposit)
            .waitWhile(deposit::isDoingWork)
            .setReversed(false)
            .splineTo(Vector2d(-58.0, -16.0).flip(blue), Math.toRadians(180.0).flip(blue))
            .turn(Math.toRadians(90.0).flip(blue))
            .splineTo(Vector2d(-58.0, -29.0).flip(blue), Math.toRadians(-90.0).flip(blue))
            .build()
        return traj
    }
    fun rightAuto() : TrajectorySequence {
        val traj = theRest(robot.trajectorySequenceBuilder(startingPosition()))
            .splineTo(allianceHub.center.polarAdd(25.0, Math.toRadians(150.0).flip(blue)), allianceHub.center)
            .dump(deposit)
            .waitWhile(deposit::isDoingWork)
            .setReversed(false)
            .splineTo(Vector2d(-58.0, -16.0).flip(blue), Math.toRadians(180.0).flip(blue))
            .turn(Math.toRadians(90.0).flip(blue))
            .splineTo(Vector2d(-58.0, -29.0).flip(blue), Math.toRadians(-90.0).flip(blue))
            .build()
        return traj
    }
}