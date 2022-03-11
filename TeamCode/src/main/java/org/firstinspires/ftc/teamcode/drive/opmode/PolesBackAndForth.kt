package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.auto.*
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.relocalizer.Relocalizer
import org.firstinspires.ftc.teamcode.roadrunnerext.flip
import org.firstinspires.ftc.teamcode.util.Async
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context.alliance
import org.firstinspires.ftc.teamcode.util.field.OpModeType

/*
* Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
* classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
* dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
* to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
* using the RC phone or http://192.168.43.1:8080/dash if you are using the Control Hub. Once
* you've successfully connected, start the program, and your robot will begin moving forward and
* backward. You should observe the target position (green) and your pose estimate (blue) and adjust
* your follower PID coefficients such that you follow the target position as accurately as possible.
* If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
* If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
* These coefficients can be tuned live in dashboard.
*
* This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
* is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
*/
@Config
@TeleOp(group = "drive")
@Disabled
class PolesBackAndForth : LinearOpMode() {
    companion object {
        @JvmField var blue = false
        @JvmField var inGain = 29.0
        @JvmField var outGain = 34.0
        @JvmField var waitTime = 0.3
    }
    override fun runOpMode() {
        val robot = Robot<Any>(this)
        robot.polesDebug = true
        waitForStart()
        robot.poseEstimate = Pose2d(-16.1417 / 2, -55.0, Math.toRadians(0.0))
        Async.start {
            while (opModeIsActive()) {
                robot.relocalizer.updatePoseEstimate(
                    Relocalizer.Sensor.FRONT_LEFT,
                    Relocalizer.Sensor.RIGHT
                )
            }
        }
        while (opModeIsActive() && !isStopRequested) {
            alliance = if (blue) {
                Alliance.BLUE
            } else {
                Alliance.RED
            }
            robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(Pose2d(-16.1417 / 2, -55.0, Math.toRadians(0.0)).flip(blue))
                    .setReversed(false)
                    .splineTo(Vector2d(inGain, -55.0).flip(blue), Math.toRadians(0.0))
                    .increaseGains(Robot.loweredVelo)
                    .carouselOn(robot.carousel)
                    .splineToConstantHeading(Vector2d(outGain, -55.0).flip(blue), Math.toRadians(0.0))
                    .defaultGains()
                    .carouselOff(robot.carousel)
                    .splineTo(Vector2d(50.0, -55.0).flip(blue), Math.toRadians(0.0))
                    .waitSeconds(waitTime)
                    .relocalize(robot)
                    .setReversed(true)
                    .splineTo(Vector2d(outGain, -55.0).flip(blue), Math.toRadians(180.0))
                    .carouselOn(robot.carousel)
                    //.increaseGains(Robot.GainMode.GAINED)
                    .splineTo(Vector2d(inGain, -55.0).flip(blue), Math.toRadians(180.0))
                    //.defaultGains()
                    .carouselOff(robot.carousel)
                    .splineTo(Vector2d(-16.1417 / 2, -55.0).flip(blue), Math.toRadians(180.0))
                    .build()
            )
        }
    }


}