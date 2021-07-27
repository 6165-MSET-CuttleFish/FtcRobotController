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

@Autonomous(name = "RED_SAFE_POWERSHOTS", group = "red")
public class RedSafePowerShots extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    Gunner gunner;
    Turret turret;
    Magazine magazine;
    Intake intake;
    WobbleArm wobbleArm;
    Claw claw;
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
                .addDisplacementMarker(() -> wobbleArm.setState(WobbleArm.State.DOWN))
                .build();
        Trajectory powershots = robot.trajectoryBuilder(forward.end())
                .addDisplacementMarker(() -> shooter.setState(Shooter.State.POWERSHOTS))
                .splineTo(Robot.pwrShotLocal(), 0)
                .build();
        TrajectorySequence wobbleDrop = robot.trajectorySequenceBuilder(powershots.end())
                .lineToLinearHeading(new Pose2d(59.5275, -10.7, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(59.5275, -50, Math.toRadians(-90)))
                .turn(Math.toRadians(180))
                .build();
        Trajectory bouncebacks = robot.trajectoryBuilder(wobbleDrop.end())
                .addDisplacementMarker(() -> {
                    shooter.setState(Shooter.State.CONTINUOUS);
                    magazine.magMacro();
                })
                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                .build();
        Trajectory park = robot.trajectoryBuilder(bouncebacks.end())
                .lineTo(new Vector2d(12, -20))
                .build();
        boolean foundRings = false;

        waitForStart();

        if (!foundRings) {
            robot.followTrajectory(forward);
            turret.setTargetAngle(Math.toRadians(100));
            robot.waitForActionsCompleted();
            turret.setState(Turret.State.IDLE);
            robot.followTrajectory(powershots);
            shooter.powerShots();
            robot.waitForActionsCompleted();
            shooter.setState(Shooter.State.IDLE);
            robot.followTrajectorySequence(wobbleDrop);
            wobbleArm.dropMacro();
            robot.waitForActionsCompleted();
            robot.followTrajectory(bouncebacks);
            gunner.shoot(3);
            robot.waitForActionsCompleted();
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
            robot.followTrajectorySequence(wobbleDrop);
            wobbleArm.dropMacro();
            robot.waitForActionsCompleted();
            robot.followTrajectory(bouncebacks);
            shooter.powerShots();
            robot.waitForActionsCompleted();
            robot.followTrajectory(park);
        }
        while (opModeIsActive()) {
            robot.update();
        }
    }
}
