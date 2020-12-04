package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

@Autonomous
public class TourneyAuto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 14, 24, 0,18, 18);
        robot.autoInit();
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        robot.scan();
        robot.discs = 4;
        robot.goTo(new Coordinate(Robot.position.x + 20, Robot.position.y - 2), 0.7, 0, 0);
        dumpWobble();
        robot.unlockIntake();
        robot.launcher.setFlyWheel(0.8);
        robot.goTo(Robot.pwrShotLocals[1], 0.5,0, 0.5);
        for(int i = 0; i < Robot.pwrShots.length; i++){
            robot.launcher.flapDown();
            robot.turnTo(Robot.pwrShots[i], 0.2);
            robot.launcher.singleRound();
        }
        //sleep(5000);

        robot.launcher.setFlyWheel(0);
        //robot.orient(0, 0.5);
        robot.goTo(Robot.leftWobble, 0.6, 0, 0.4);
        robot.goTo(new Coordinate(Robot.position.x - 8, Robot.position.y), 0.2, 0, 0);
        robot.grab();
        sleep(350);
        robot.wobbleArmUp();
        if(robot.discs != 0) {
            getMoreRings();
        }
        dumpWobble();
        Coordinate homePos = new Coordinate(88, robot.position.y);
        if(robot.discs != 4){
            homePos.y = robot.position.y + 15;
        }
        robot.goTo(homePos, 0.5, 0, 0);
        robot.position.stop();
    }
    public void case0(){
        robot.goTo(Robot.A, 0.5, 0, 0);
        robot.wobbleArmDown();
        sleep(1000);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x + 15, Robot.position.y), 0.5, 0, 0);
        //robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 15), 0.7, 0, 0);

    }
    public void case1(){
        robot.goTo(Robot.B, 0.7, 0, 0);
        robot.wobbleArmDown();
        sleep(1000);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x + 15, Robot.position.y), 0.7, 0, 0);
    }
    public void case4(){
        robot.goTo(Robot.C, 0.8, Math.toRadians(180), 0.5);
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        sleep(300);
        robot.goTo(new Coordinate(Robot.position.x - 10, Robot.position.y), 0.8, 0, 0);
    }
    public void dumpWobble(){
        if(robot.discs == 4){
            case4();
        }
        else if(robot.discs == 1){
            case1();
        }
        else{
            case0();
        }

        //robot.wobbleArmUp();
    }
    public void getMoreRings(){
        Coordinate rings = new Coordinate(47, 32);
        robot.intake(1);
        robot.goTo(rings, 1, 0, 0.6);
        robot.launcher.setOnlyFlyWheel(1);
        robot.goTo(new Coordinate(67, Robot.hiGoal.y), 0.7, 0, 0.6);
        sleep(2500);
        robot.intake(-1);
        sleep(100);
        robot.launcher.magazineShoot();
        robot.launcher.setFlyWheel(0);
        robot.intake(0);
    }
}
