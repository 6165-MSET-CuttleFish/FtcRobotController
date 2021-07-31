package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.Side;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.Components.Details.robotPose;
import static org.firstinspires.ftc.teamcode.Components.Robot.stackHeight;

@Autonomous(name = "BLUE_SAFE_POWERSHOTS", group = "blue")
public class BlueSafePowerShots extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    Gunner gunner;
    Turret turret;
    Magazine magazine;
    Intake intake;
    WobbleArm wobbleArm;
    Claw claw;

    Trajectory powershots;
    TrajectorySequence wobbleDrop4, wobbleDrop1, wobbleDrop0;
    Trajectory bouncebacks4, bouncebacks1, bouncebacks0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, new Pose2d(-61.5975, -16.8475, 0), OpModeType.AUTO, Side.RED);
        shooter = robot.shooter;
        intake = robot.intake;
        wobbleArm = robot.wobbleArm;
        turret = shooter.turret;
        gunner = shooter.gunner;
        magazine = shooter.magazine;
        claw = wobbleArm.claw;
        Trajectory forward = robot.trajectoryBuilder(robotPose)
                .forward(10)
                .addTemporalMarker(0.3, () -> wobbleArm.setState(WobbleArm.State.DOWN))
                .build();
        powershots = robot.trajectoryBuilder(forward.end())
                .addDisplacementMarker(() -> shooter.setState(Shooter.State.POWERSHOTS))
                .splineTo(Robot.pwrShotLocal(), 0)
                .build();
        //wobble drop
        wobbleDrop0 = robot.trajectorySequenceBuilder(powershots.end())
                .lineToLinearHeading(new Pose2d(58.5275, 3, Math.toRadians(90)))
                .splineTo(Robot.dropZonesPS()[0].vec(), Math.toRadians(90))
                .turn(Math.toRadians(180))
                .build();
        wobbleDrop1 = robot.trajectorySequenceBuilder(powershots.end())
                .lineToLinearHeading(new Pose2d(58.5275, 3, Math.toRadians(90)))
                .splineTo(Robot.dropZonesPS()[1].vec(), Math.toRadians(-90))
                .turn(Math.toRadians(180))
                .build();
        wobbleDrop4 = robot.trajectorySequenceBuilder(powershots.end())
                .lineToLinearHeading(new Pose2d(58.5275, 3, Math.toRadians(90)))
                .splineTo(Robot.dropZonesPS()[2].vec(), Math.toRadians(-90))
                .turn(Math.toRadians(180))
                .build();
        // bouncebacks
        bouncebacks0 = robot.trajectoryBuilder(wobbleDrop0.end())
                .addDisplacementMarker(() -> {
                    shooter.setState(Shooter.State.CONTINUOUS);
                    intake.setPower(0);
                    magazine.magMacro();
                })
                .splineTo(new Vector2d(15, 20), Math.toRadians(180))
                .splineTo(new Vector2d(-5.8, 20), Math.toRadians(180))
                .build();
        bouncebacks1 = robot.trajectoryBuilder(wobbleDrop1.end())
                .addDisplacementMarker(() -> {
                    shooter.setState(Shooter.State.CONTINUOUS);
                    intake.setPower(0);
                    magazine.magMacro();
                })
                .splineTo(new Vector2d(15, 20), Math.toRadians(180))
                .splineTo(new Vector2d(-5.8, 20), Math.toRadians(180))
                .build();
        bouncebacks4 = robot.trajectoryBuilder(wobbleDrop4.end())
                .addDisplacementMarker(() -> {
                    shooter.setState(Shooter.State.CONTINUOUS);
                    intake.setPower(0);
                    magazine.magMacro();
                })
                .splineTo(new Vector2d(15, 20), Math.toRadians(180))
                .splineTo(new Vector2d(-5.8, 20), Math.toRadians(180))
                .build();
        Trajectory park = robot.trajectoryBuilder(bouncebacks4.end())
                .lineTo(new Vector2d(12, 20))
                .build();
        boolean foundRings = false;
        sleep(500);
        while (!opModeIsActive() && !isStopRequested()) {
            robot.scan();
            telemetry.addData("ring", stackHeight.toString());
            telemetry.update();
        }
        robot.setPoseEstimate(robotPose);

        waitForStart();

        intake.dropIntake();
        claw.grab();

        if (!foundRings) {
            robot.followTrajectory(forward);
            //turret.setTargetAngle(Math.toRadians(150));
            robot.waitForActionsCompleted();
            turret.setState(Turret.State.IDLE);
            robot.followTrajectory(powershots);
            shooter.powerShots();
            robot.waitForActionsCompleted();
            shooter.setState(Shooter.State.IDLE);
            intake.setPower(1);
            wobbleArm.setState(WobbleArm.State.MID);
            robot.followTrajectorySequence(getWobbleDrop());
            wobbleArm.dropMacro();
            robot.waitForActionsCompleted();
            robot.followTrajectory(getBounceBacks());
            gunner.shoot(3);
            robot.waitForActionsCompleted();
            shooter.setState(Shooter.State.IDLE);
            robot.followTrajectory(park);
        } else {
            robot.followTrajectory(forward);
            turret.setTargetAngle(Math.toRadians(100));
            robot.waitForActionsCompleted();
            turret.setState(Turret.State.IDLE);
            robot.followTrajectory(powershots);
            shooter.powerShots();
            robot.waitForActionsCompleted();
            shooter.setState(Shooter.State.IDLE);
            robot.followTrajectorySequence(wobbleDrop4);
            wobbleArm.dropMacro();
            robot.waitForActionsCompleted();
            robot.followTrajectory(bouncebacks4);
            shooter.powerShots();
            robot.waitForActionsCompleted();
            robot.followTrajectory(park);
        }
        while (opModeIsActive()) {
            robot.update();
        }
    }
    private TrajectorySequence getWobbleDrop() {
        switch (stackHeight){
            case ZERO: return wobbleDrop0;
            case ONE: return wobbleDrop1;
            default: return wobbleDrop4;
        }
    }

    private Trajectory getBounceBacks() {
        switch (stackHeight){
            case ZERO: return bouncebacks0;
            case ONE: return bouncebacks1;
            default: return bouncebacks4;
        }
    }
}
