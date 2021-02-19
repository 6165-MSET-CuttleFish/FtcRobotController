package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

@Autonomous
public class TourneyAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;
    double powerReturn(){
        if(robot.discs == 0){
            return 0.35;
        } else if(robot.discs == 1){
            return 0.3;
        } else {
            return 0.44;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 9, 25, 0, 18, 18, telemetry, this::opModeIsActive);
        robot.autoInit();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.tiltUp();
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
       // if(robot.discs != 0) {
            robot.goTo(new Coordinate(Robot.position.x + 13, Robot.position.y - 8), 0.8, 0, 0);
            if(robot.discs != 0)
            robot.goTo(new Coordinate(Robot.position.x + 30, Robot.position.y - 2), 0.8, 0, 0);
        //}
        if (robot.discs == 4) {
            targetPos = Robot.newC;
        } else if (robot.discs == 1) {
            targetPos = Robot.newB;
        } else {
            targetPos = Robot.newA;
        }

        try {
            if(robot.discs == 4)
            robot.goTo(targetPos, 0.8, Math.toRadians(90), 0.4, () -> robot.wobbleArmDown());
            else robot.goTo(targetPos, 0.6, Math.toRadians(90), 0.4, () -> robot.wobbleArmDown());
        } catch (Exception e) {
            e.printStackTrace();
        }
        targetPos.add(-5, -3);
        robot.release();
        sleep(200);
        robot.unlockIntake();
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
        try {
            robot.goTo(Robot.pwrShotLocals[0],powerReturn(), Math.toRadians(0), 0.3, () -> {
                robot.launcher.setOnlyFlyWheel(0.4);
                robot.launcher.tiltUp();
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
            robot.launcherTurnTo(pwrShot, 0.23);
            robot.launcher.singleRound();
        }
        sleep(400);
        robot.launcher.setFlyWheel(0);
        try {
            robot.goTo(Robot.leftWobble, 0.48, 0, 0.5, () -> {
                if(Robot.position.distanceTo(Robot.leftWobble) < 7)
                    robot.grab();
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
        //sleep(600);
        robot.wobbleArmUp();
        if (robot.discs != 0) {
            getMoreRings();
        }
        if (robot.discs == 4) {
            targetPos = Robot.C;
        } else if (robot.discs == 1) {
            targetPos = Robot.newB;
        } else {
            targetPos = Robot.newA;
        }
        if(robot.discs == 4) {
            try {
                robot.goTo(targetPos, 0.7, Math.toRadians(160), 0.4, () -> robot.wobbleArmDown());
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            try {
                robot.goTo(targetPos, 0.7, Math.toRadians(90), 0.4, () -> robot.wobbleArmDown());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        robot.release();
        sleep(200);
        robot.wobbleArmUp();
        Coordinate homePos = new Coordinate(88, Robot.position.y);
        if(robot.discs != 0) {
            robot.goTo(homePos, 0.6, Math.toRadians(180), 0);
        } else {
            robot.goTo(new Coordinate(80, 48), 0.5, 0, 0.5);
        }
        sleep(600);
        Robot.position.stop();
    }
    private void getMoreRings() {
        robot.goTo(new Coordinate(Robot.position.x - 8, Robot.position.y - 13), 0.7, Math.toRadians(-10), 0.5);
        Coordinate rings2 = new Coordinate(Robot.position.x + 15.2, Robot.position.y);
        robot.intake(1);
        if(robot.discs == 4) {
            robot.goTo(rings2, 0.28, 0, 0.3);
            robot.goTo(new Coordinate(Robot.position.x, Robot.position.y - 12), 0.6, Math.toRadians(-8), 0.3);
        }
        else {
            robot.goTo(rings2, 0.3, 0, 0.3);

        }
        //robot.launcher.flapUp();
        robot.launcher.flapDown();
        robot.launcher.setOnlyFlyWheel(0.49);
        robot.intake(0);
        robot.goTo(new Coordinate(55, Robot.hiGoal.y), 0.6, Math.toRadians(-5), 0.4);
        robot.intake(1);
        sleep(700);
        robot.launcher.tiltUp();
        robot.intake(0);
        sleep(400);
        if (robot.discs == 4) {
            robot.launcher.magazineShoot();
        } else {
            robot.launcher.singleRound();
            sleep(300);
            robot.launcher.singleRound();
            sleep(400);
        }
        robot.launcher.setFlyWheel(0);
    }
}
