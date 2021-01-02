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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 9, 25, 0, 18, 18);
        robot.autoInit();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        robot.scan();
        telemetry.addData("Stack Height", robot.height);
        telemetry.addData("Discs", robot.discs);
        telemetry.update();
        robot.goTo(new Coordinate(Robot.position.x + 30, Robot.position.y - 5), 1, 0, 0);
        //robot.goTo(new Coordinate(Robot.position.x + 5, Robot.position.y - 10), 1, 0, 0);
        if (robot.discs == 4) {
            targetPos = Robot.newC;
        } else if (robot.discs == 1) {
            targetPos = Robot.newB;
        } else {
            targetPos = Robot.newA;
        }

        robot.goTo(targetPos, 0.7, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(550);
        robot.release();
        sleep(200);
        robot.unlockIntake();
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
        robot.launcher.setFlyWheel(0.75);
        robot.goTo(Robot.pwrShotLocals[1], 0.6, 0, 0.3);
        sleep(300);
        for (int i = 0; i < Robot.pwrShots.length; i++) {
            robot.launcher.flapDown();
            robot.launcherturnTo(Robot.pwrShots[i], 0.22);
            robot.launcher.singleRound();
        }
        robot.launcher.setFlyWheel(0);
        robot.goTo(Robot.leftWobble, 0.55, 0, 0.5);
        robot.goTo(new Coordinate(Robot.position.x - 3, Robot.position.y), 0.2, 0, 0.5);
        robot.grab();
        sleep(500);
        robot.wobbleArmUp();
        if (robot.discs != 0) {
            getMoreRings();
        }
        if (robot.discs == 4) {
            targetPos = Robot.C;
            case4();
        } else if (robot.discs == 1) {
            targetPos = Robot.B;
            case1();
        } else {
            targetPos = Robot.A;
            case0();
        }
        Coordinate homePos = new Coordinate(88, robot.position.y);
        robot.goTo(homePos, 0.7, 0, 0);
        while (opModeIsActive()) {
        }
        robot.position.stop();
    }

    public void newcase0() {
        robot.goTo(Robot.newA, 0.7, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(450);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
    }

    public void newcase1() {
        robot.goTo(Robot.newB, 0.7, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(450);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
    }

    public void newcase4() {
        robot.goTo(Robot.newC, 0.8, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(450);
        robot.release();
        sleep(200);
        //robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
    }

    public void case0() {
        robot.goTo(Robot.A, 0.6, Math.toRadians(-180), 0.5);
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        sleep(200);
        //robot.goTo(new Coordinate(Robot.position.x + 10, Robot.position.y), 1, 0, 0);
        //robot.goTo(new Coordinate(Robot.position.x , Robot.position.y + 10), 1, 0, 0);

        //robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 15), 0.7, 0, 0);

    }

    public void case1() {
        robot.goTo(Robot.B, 0.7, Math.toRadians(-180), 0.5);
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        sleep(200);
//        robot.goTo(new Coordinate(Robot.position.x + 10, Robot.position.y), 1, Math.toRadians(-180), 0);
//        robot.goTo(new Coordinate(Robot.position.x , Robot.position.y + 10), 1, 0, 0);

    }

    public void case4() {
        Coordinate slowDownPt = new Coordinate(Robot.C);
        slowDownPt.addX(-20);
        slowDownPt.addY(10);
        robot.goTo(slowDownPt, 0.7, Math.toRadians(150), 0.5);
        robot.wobbleArmDown();
        robot.goTo(Robot.C, 0.7, Math.toRadians(150), 0.4);
        robot.release();
        sleep(500);
        //robot.goTo(new Coordinate(Robot.position.x - 10, Robot.position.y), 0.8, 0, 0);
    }

    public void getMoreRings() {
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y - 15), 1, 0, 0.5);
        Coordinate rings2 = new Coordinate(Robot.position.x + 16, Robot.position.y);
        robot.intake(1);
        robot.goTo(rings2, 0.3, 0, 0.3);
        robot.launcher.setOnlyFlyWheel(1);
        robot.launcher.flapUp();
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y - 12), 0.7, 0, 0.3);
        robot.goTo(new Coordinate(50, Robot.hiGoal.y), 0.7, 0, 0.4);
        robot.intake(-1);
        sleep(1000);
        if (robot.discs == 4) {
            robot.launcher.magazineShoot();
        } else {
            robot.launcher.singleRound();
        }
        robot.launcher.setFlyWheel(0);
        robot.intake(0);
    }
}
