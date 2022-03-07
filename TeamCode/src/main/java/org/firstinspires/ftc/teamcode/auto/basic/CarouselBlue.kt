package org.firstinspires.ftc.teamcode.auto.basic


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
        waitForStart()
        robot.scan()
        val trajectoryBuilder =
            robot.trajectorySequenceBuilder(startingPosition()).setReversed(true)
        trajectoryBuilder
            .setVelConstraint(getVelocityConstraint(15.0, Math.PI,15.0))
            .splineTo(allianceHub.center.polarAdd(20.0,Math.toRadians(-100.0).flip(blue)), allianceHub.center)
            .waitSeconds(0.5)
            .waitWhile(deposit::isDoingWork)
            .dump(deposit)
            .resetConstraints()
            .setReversed(false)
            .forward(7.0)
            .turn(Math.toRadians(240.0))
            .splineTo(Vector2d(-50.0,-46.0).flip(blue),Math.toRadians(175.0).flip(blue))
            .setReversed(true)
            .setVelConstraint(getVelocityConstraint(10.0, Math.PI,15.0))
            .splineTo(carouselVec.center.polarAdd(13.0, Math.toRadians(45.0).flip(blue)), carouselVec.center, Pose2d())
            .UNSTABLE_addTemporalMarkerOffset(0.0){
                carousel.setPower(-0.4)
            }
            .waitSeconds(2.0)
            .setReversed(false)
            .carouselOff(carousel)
            .resetConstraints()
            .splineTo(Vector2d(-56.0, -24.0).flip(blue), Math.toRadians(90.0).flip(blue))

        val trajectorySequence = trajectoryBuilder
            .build()
        robot.turnOffVision()
        robot.followTrajectorySequence(trajectorySequence)
    }
}