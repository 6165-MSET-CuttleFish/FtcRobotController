package org.firstinspires.ftc.teamcode.Auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Components.*
import org.firstinspires.ftc.teamcode.Components.Robot.*
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.sequencesegment.FutureSegment

@Autonomous(name = "BLUE_AGGRESSIVE", group = "blue")
class BetterPath : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var shooter: Shooter
    lateinit var gunner: Gunner
    lateinit var turret: Turret
    lateinit var magazine: Magazine
    lateinit var intake: Intake
    lateinit var wobbleArm: WobbleArm
    lateinit var claw: Claw
    lateinit var mainSequence: TrajectorySequence
    private val wobbleDrop = FutureSegment(Pose2d(), Pose2d())
    lateinit var powerShots: TrajectorySequence
    lateinit var bouncebacks: TrajectorySequence
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot = Robot(this, Pose2d(-61.5975, 23.64, 0.0), OpModeType.AUTO, Side.BLUE)
        shooter = robot.shooter
        intake = robot.intake
        wobbleArm = robot.wobbleArm
        turret = shooter.turret
        gunner = shooter.gunner
        magazine = shooter.magazine
        claw = wobbleArm.claw
        shooter.state = Shooter.State.EMPTY_MAG
        generatePaths()
        robot.poseEstimate = Details.robotPose
        telemetry.addData("Ready", true)
        telemetry.update()
        while (!opModeIsActive() && !isStopRequested) {
            robot.scan()
            telemetry.addData("ring", stackHeight.toString())
            telemetry.update()
        }
        waitForStart()
        Async.start(this::fulfillFutures)
        robot.followTrajectorySequence(mainSequence)
    }

    private fun fulfillFutures() {
        wobbleDrop.trajectory = robot.futureBuilder(wobbleDrop)
            .setReversed(true)
            .splineTo(Vector2d(58.0, 5.0), Math.toRadians(90.0))
            .splineTo(robot.dropZone.vec(), robot.dropZone.heading)
            .UNSTABLE_addTemporalMarkerOffset(0.4) {
                magazine.magMacro()
                shooter.state = Shooter.State.CONTINUOUS
            }
            .splineTo(Vector2d(-5.0, 16.7), Math.toRadians(180.0))
            .build()
    }

    private fun generatePaths() {
        mainSequence = robot.trajectorySequenceBuilder(Details.robotPose)
            .lineToSplineHeading(Pose2d(46.0, 16.8475))
            .splineTo(Vector2d(58.0, 10.0), Math.toRadians(-90.0))
            .splineTo(Vector2d(58.0, -17.0), Math.toRadians(-90.0))
            .waitCondition { !robot.isBusy }
            .addFutureTrajectory(wobbleDrop, Pose2d(-5.0, 16.7, Math.toRadians(180.0)))
            .addDisplacementMarker { intake.setPower(1.0) }
            .lineTo(Vector2d(-55.0, 16.7)) // Intake starter rings
            .setReversed(true)
            .prepShooter(0.5, robot)
            .splineTo(pwrShotLocal(), 0.0)
            .addDisplacementMarker { intake.setPower(1.0) }
            .lineToSplineHeading(Pose2d(40.0, 3.0, Math.toRadians(0.0)))
            .splineTo(Vector2d(58.5275, 40.0), Math.toRadians(90.0))
            .setReversed(true)
            .prepShooter(0.5, robot)
            .splineTo(Vector2d(20.0, 16.0), Math.toRadians(180.0))
            .splineTo(Vector2d(-5.8, 17.0), Math.toRadians(180.0))
            .lineTo(Vector2d(12.0, 17.0))
            .build()
    }
}