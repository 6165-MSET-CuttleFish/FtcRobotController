package org.firstinspires.ftc.teamcode.Auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.vision.UGContourRingPipeline.Height
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Components.*
import org.firstinspires.ftc.teamcode.Components.Robot.stackHeight
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.sequencesegment.FutureSegment

@Autonomous(name = "BLUE_AGGRESSIVE", group = "blue")
class BlueAggressive : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var shooter: Shooter
    lateinit var gunner: Gunner
    lateinit var turret: Turret
    lateinit var magazine: Magazine
    lateinit var intake: Intake
    lateinit var wobbleArm: WobbleArm
    lateinit var claw: Claw
    lateinit var mainSequence: TrajectorySequence
    lateinit var wobbleDrop4: Trajectory
    lateinit var wobbleDrop1: Trajectory
    lateinit var wobbleDrop0: Trajectory
    lateinit var shootBonked4: TrajectorySequence
    lateinit var shootBonked1: TrajectorySequence
    lateinit var shootBonked0: TrajectorySequence
    lateinit var powerShots: TrajectorySequence
    lateinit var bouncebacks: TrajectorySequence
    lateinit var park: TrajectorySequence
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
        intake.dropIntake()
        robot.followTrajectorySequence(mainSequence)
        robot.followTrajectory(wobbleDrop)
        wobbleArm.dropMacro()
        intake.setPower(0.0)
        robot.waitForActionsCompleted()
        robot.followTrajectorySequence(shootBonked)
        gunner.shoot(3)
        robot.waitForActionsCompleted()
        robot.followTrajectorySequence(powerShots)
        shooter.powerShots()
        robot.waitForActionsCompleted()
        shooter.state = Shooter.State.IDLE
        robot.followTrajectorySequence(bouncebacks)
        gunner.shoot(3)
        robot.waitForActionsCompleted()
        shooter.state = Shooter.State.IDLE
        robot.followTrajectorySequence(park)
    }

    private fun generatePaths() {
        mainSequence = robot.trajectorySequenceBuilder(Details.robotPose)
                .lineToSplineHeading(Pose2d(46.0, 16.8475))
                .splineTo(Vector2d(58.0, 10.0), Math.toRadians(-90.0))
                .splineTo(Vector2d(58.0, -17.0), Math.toRadians(-90.0))
                .build()
        // Wobble Drop
        wobbleDrop0 = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(Vector2d(58.0, 5.0), Math.toRadians(90.0))
                .splineTo(Robot.dropZonesPS()[0].vec(), Robot.dropZonesPS()[0].heading)
                .build()
        wobbleDrop1 = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(Vector2d(58.0, 5.0), Math.toRadians(90.0))
                .splineTo(Robot.dropZonesPS()[1].vec(), Robot.dropZonesPS()[1].heading)
                .build()
        wobbleDrop4 = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(Vector2d(58.0, 5.0), Math.toRadians(90.0))
                .splineTo(Robot.dropZonesPS()[2].vec(), Robot.dropZonesPS()[2].heading)
                .build()
        // Shoot Bonked
        shootBonked0 = robot.trajectorySequenceBuilder(wobbleDrop0.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4) {
                    magazine.magMacro()
                    shooter.state = Shooter.State.CONTINUOUS
                }
                .splineTo(Vector2d(-5.0, 16.7), Math.toRadians(180.0))
                .build()
        shootBonked1 = robot.trajectorySequenceBuilder(wobbleDrop1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4) {
                    magazine.magMacro()
                    shooter.state = Shooter.State.CONTINUOUS
                }
                .splineTo(Vector2d(-5.0, 16.7), Math.toRadians(180.0))
                .build()
        shootBonked4 = robot.trajectorySequenceBuilder(wobbleDrop4.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4) {
                    magazine.magMacro()
                    shooter.state = Shooter.State.CONTINUOUS
                }
                .splineTo(Vector2d(-5.0, 16.7), Math.toRadians(180.0))
                .build()
        // PowerShots
        powerShots = robot.trajectorySequenceBuilder(shootBonked4.end())
                .addDisplacementMarker { intake.setPower(1.0) }
                .lineTo(Vector2d(-55.0, 16.7)) // Intake starter rings
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    intake.setPower(0.0)
                    magazine.magMacro()
                    shooter.state = Shooter.State.POWERSHOTS
                }
                .splineTo(Robot.pwrShotLocal(), 0.0)
                .build()
        bouncebacks = robot.trajectorySequenceBuilder(powerShots.end())
                .addDisplacementMarker { intake.setPower(1.0) }
                .lineToSplineHeading(Pose2d(40.0, 3.0, Math.toRadians(0.0)))
                .splineTo(Vector2d(58.5275, 40.0), Math.toRadians(90.0))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    intake.setPower(0.0)
                    magazine.magMacro()
                    shooter.state = Shooter.State.CONTINUOUS
                }
                .splineTo(Vector2d(20.0, 16.0), Math.toRadians(180.0))
                .splineTo(Vector2d(-5.8, 17.0), Math.toRadians(180.0))
                .build()
        park = robot.trajectorySequenceBuilder(bouncebacks.end())
                .lineTo(Vector2d(12.0, 17.0))
                .build()
    }

    private val wobbleDrop: Trajectory
        get() = when (stackHeight) {
            Height.ZERO -> wobbleDrop0
            Height.ONE -> wobbleDrop1
            else -> wobbleDrop4
        }
    private val shootBonked: TrajectorySequence
        get() = when (stackHeight) {
            Height.ZERO -> shootBonked0
            Height.ONE -> shootBonked1
            else -> shootBonked4
        }
}