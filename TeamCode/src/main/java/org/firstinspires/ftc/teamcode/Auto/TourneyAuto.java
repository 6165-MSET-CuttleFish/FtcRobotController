package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import java.io.Console;

@Autonomous
public class TourneyAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 9, 25, 0, 18, 18, this::opModeIsActive);
        robot.autoInit();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        while(!opModeIsActive()){
            robot.scan();
            telemetry.addData("Stack Height", robot.height);
            telemetry.addData("Discs", robot.discs);
            telemetry.update();
        }
        waitForStart();
        robot.scan();
        robot.turnOffVision();
        telemetry.addData("Stack Height", robot.height);
        telemetry.addData("Discs", robot.discs);
        telemetry.update();
        if(robot.discs != 0) robot.goTo(new Coordinate(Robot.position.x + 15, Robot.position.y - 8), 0.9, 0, 0);
        //robot.goTo(new Coordinate(Robot.position.x + 5, Robot.position.y - 10), 1, 0, 0);
        if (robot.discs == 4) {
            targetPos = Robot.newC;
        } else if (robot.discs == 1) {
            targetPos = Robot.newB;
        } else {
            targetPos = Robot.newA;
        }

        try {
            robot.goTo(targetPos, 0.8, Math.toRadians(90), 0.4, () -> robot.wobbleArmDown());
        } catch (Exception e) {
            e.printStackTrace();
        }
        robot.release();
        sleep(200);
        robot.unlockIntake();
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
        try {
            robot.goTo(Robot.pwrShotLocals[1], 0.5, Math.toRadians(0), 0.3, () -> {
                robot.launcher.setFlyWheel(0.5);
                robot.wingsIn();
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
        telemetry.addData("x", Robot.position.x);
        telemetry.addData("y", Robot.position.y);
        telemetry.update();
        sleep(300);
        for (Coordinate pwrShot : Robot.pwrShots) {
            robot.launcher.flapDown();
            robot.launcherturnTo(pwrShot, 0.21);
            robot.launcher.singleRound();
        }
        robot.launcher.setFlyWheel(0);
        try {
            robot.goTo(Robot.leftWobble, 0.48, 0, 0.5, () -> {
                if(Robot.position.distanceTo(Robot.leftWobble) < 7)
                    robot.grab();
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
        sleep(600);
        robot.wobbleArmUp();
        if (robot.discs != 0) {
            getMoreRings();
        }
        if (robot.discs == 4) {
            targetPos = Robot.C;
        } else if (robot.discs == 1) {
            targetPos = Robot.B;
        } else {
            targetPos = Robot.A;
        }
        try {
            robot.goTo(targetPos, 0.7, Math.toRadians(140), 0.4, () -> robot.wobbleArmDown());
        } catch (Exception e) {
            e.printStackTrace();
        }
        robot.release();
        sleep(200);
        Coordinate homePos = new Coordinate(88, Robot.position.y);
        if(robot.discs != 0) {
            robot.goTo(homePos, 0.6, Math.toRadians(180), 0);
        }
        robot.wobbleArmUp();
        sleep(600);
        Robot.position.stop();
    }
    public void getMoreRings() {
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y - 16), 0.7, 0, 0.5);
        Coordinate rings2 = new Coordinate(Robot.position.x + 14.2, Robot.position.y);
        robot.intake(0.7);
        if(robot.discs == 4) {
            robot.goTo(rings2, 0.25, 0, 0.3);
        }
        else {
            robot.goTo(rings2, 0.7, 0, 0.3);
        }
        //robot.launcher.flapUp();
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y - 12), 0.7, Math.toRadians(-30), 0.3);
        robot.goTo(new Coordinate(52, Robot.hiGoal.y), 0.7, Math.toRadians(-20), 0.4);
        robot.launcher.setOnlyFlyWheel(0.8);
        robot.intake(0);
        sleep(800);
        if (robot.discs == 4) {
            robot.launcher.magazineShoot();
        } else {
            robot.launcher.singleRound();
        }
        robot.launcher.setFlyWheel(0);
        robot.intake(0);
    }
}
