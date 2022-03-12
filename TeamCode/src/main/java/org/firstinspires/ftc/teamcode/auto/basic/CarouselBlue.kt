package org.firstinspires.ftc.teamcode.auto.basic


import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.allianceHub
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.carouselVec
import org.firstinspires.ftc.teamcode.drive.FrequentPositions.startingPosition
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.drive.Robot.*
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context.location
import org.firstinspires.ftc.teamcode.util.field.Context.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import org.firstinspires.ftc.teamcode.roadrunnerext.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.Line
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import kotlin.Throws

@Autonomous
class CarouselBlue : LinearOpMode() {
    lateinit var robot: Robot<PathState>
    lateinit var deposit: Deposit
    lateinit var intake: Intake
    lateinit var capstone: Capstone
    lateinit var carousel: Carousel
    enum class PathState {
        INTAKING,
        DUMPING,
        IDLE,
    }
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot = Robot(this, OpModeType.AUTO, Alliance.BLUE)
        intake = robot.intake
        capstone = robot.capstone
        deposit = robot.deposit
        carousel = robot.carousel
        val blue = true
        side = Side.CAROUSEL
        robot.visionInit()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("Location", location)
            telemetry.update()
        }
        Deposit.allowLift = false
        val trajectorySequence =
            robot.trajectorySequenceBuilder(startingPosition())
                .back(7.0)
                .turn(Math.toRadians(90.0).flip(blue))
                .setReversed(true)
                .setVelConstraint(getVelocityConstraint(20.0, Math.PI,15.0))
                .splineTo(Vector2d(-55.0, -53.0).flip(blue), Math.toRadians(250.0).flip(blue))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    carousel.setPower(0.9)
                }
                .waitSeconds(2.0, DriveSignal(Pose2d(-5.0, 0.0, Math.toRadians(0.0))))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    carousel.setPower(-0.0)
                }
                .resetConstraints()
                .setReversed(false)
                .setVelConstraint(getVelocityConstraint(40.0, Math.PI,15.0))
                .splineTo(Vector2d(-55.0, -27.0).flip(blue), Math.toRadians(90.0).flip(blue))
                .splineTo(Vector2d(-39.0, -8.0).flip(blue), Math.toRadians(120.0).flip(blue))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    Deposit.allowLift = true
                }
                .setReversed(true)
                .splineTo(allianceHub.center.polarAdd(20.0, Math.toRadians(160.0).flip(blue)), allianceHub.center)
                .setReversed(false)
                .splineTo(Vector2d(-58.0, -16.0).flip(blue), Math.toRadians(180.0).flip(blue))
                .turn(Math.toRadians(90.0).flip(blue))
                .splineTo(Vector2d(-58.0, -35.0).flip(blue), Math.toRadians(-90.0).flip(blue))
                .build()
        waitForStart()
        robot.scan()
        robot.turnOffVision()
        robot.followTrajectorySequence(trajectorySequence)
    }
}