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

        waitForStart();
        robot.scan();
        robot.discs = 4;
        dumpWobble();
        robot.unlockIntake();
        robot.goTo(Robot.pwrShotLocals[1], 0.5, 180, 0.5);
        for(int i = 0; i < Robot.pwrShots.length; i++){
            robot.launcher.setFlyWheel(1);
            robot.launcher.flapUp();
            robot.turnTo(Robot.pwrShots[i], 0.5);
            robot.launcher.singleRound();
        }
        robot.launcher.setFlyWheel(0);
        robot.goTo(Robot.leftWobble, 0.5, 0, 0.3);
        robot.wobbleArmDown();
        sleep(500);
        robot.grab();
        sleep(200);
        robot.wobbleArmUp();
        dumpWobble();
        Coordinate homePos = new Coordinate(80, robot.position.y);
        robot.goTo(homePos, 0.5, 0, 0);
        robot.position.stop();
    }
    public void case0(){
        robot.goTo(Robot.A, 0.5, 0, 0);
    }
    public void case1(){
        robot.goTo(Robot.B, 0.5, 0, 0);
    }
    public void case4(){
        robot.goTo(Robot.C, 0.5, Math.toRadians(180), 0.5);
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
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        robot.wobbleArmUp();
    }
    public void getMoreRings(){
        Coordinate rings = new Coordinate(47, 34);
        robot.intake(1);
        robot.goTo(rings, 0.5, Math.toRadians(Robot.position.angleTo(rings)), 0.8);
        robot.intake(0);
        robot.launcher.setFlyWheel(1);
        sleep(1000);
        robot.launcher.magazineShoot();
        robot.launcher.setFlyWheel(0);
    }
}
