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
        //if(robot.discs != 4) {
        robot.goTo(Robot.pwrShotLocals[1], 0.7, 0, 0.5);
        //}
//        else{
//            robot.goTo(Robot.pwrShotLocals[1], 0.7, Math.toRadians(180), 0.5);
//        }
        robot.launcher.setFlyWheel(1);
        for(int i = 0; i < Robot.pwrShots.length; i++){
            robot.launcher.flapDown();
            robot.turnTo(Robot.pwrShots[i], 0.5);
            robot.launcher.singleRound();
        }
        telemetry.addData("WE MADE IT", "");
        telemetry.update();
        //sleep(5000);

        robot.launcher.setFlyWheel(0);
        robot.orient(0, 0.5);
        robot.goTo(Robot.leftWobble, 0.5, 0, 0);
        robot.wobbleArmDown();
        sleep(1000);
        robot.goTo(new Coordinate(Robot.position.x - 10, Robot.position.y), 0.5, 0, 0);
        robot.grab();
        sleep(400);
        robot.wobbleArmUp();
        dumpWobble();
        Coordinate homePos = new Coordinate(80, robot.position.y);
        robot.goTo(homePos, 0.5, 0, 0);
        robot.position.stop();
    }
    public void case0(){
        robot.goTo(Robot.A, 0.7, 0, 0);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 15), 0.7, 0, 0);
    }
    public void case1(){
        robot.goTo(Robot.B, 0.7, 0, 0);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 15), 0.7, 0, 0);
    }
    public void case4(){
        robot.goTo(Robot.C, 0.7, Math.toRadians(180), 0.5);
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
        sleep(1000);
        robot.release();
        sleep(400);
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
