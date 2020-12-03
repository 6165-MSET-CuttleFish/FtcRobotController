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
        robot.goTo(new Coordinate(Robot.position.x + 20, Robot.position.y - 2), 0.5, 0, 0);
        dumpWobble();
        robot.unlockIntake();
        robot.goTo(Robot.pwrShotLocals[1], 0.5,0, 0.5);
        robot.launcher.setFlyWheel(0.8);
        sleep(300);
        for(int i = 0; i < Robot.pwrShots.length; i++){
            robot.launcher.flapDown();
            robot.turnTo(Robot.pwrShots[i], 0.2);
            robot.launcher.singleRound();
        }
        //sleep(5000);

        robot.launcher.setFlyWheel(0);
        //robot.orient(0, 0.5);
        robot.goTo(Robot.leftWobble, 0.5, 0, 0.4);
        robot.goTo(new Coordinate(Robot.position.x - 8, Robot.position.y), 0.2, 0, 0);
        robot.grab();
        sleep(500);
        robot.wobbleArmUp();
        if(robot.discs != 0) {
            getMoreRings();
        }
        dumpWobble();
        Coordinate homePos = new Coordinate(84, robot.position.y);
        robot.goTo(homePos, 0.5, 0, 0);
        robot.position.stop();
    }
    public void case0(){
        robot.goTo(Robot.A, 0.7, 0, 0);
        robot.wobbleArmDown();
        sleep(1000);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x + 15, Robot.position.y), 0.7, 0, 0);
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
        robot.goTo(Robot.C, 0.7, Math.toRadians(180), 0.5);
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        sleep(300);
        robot.goTo(new Coordinate(Robot.position.x - 10, Robot.position.y), 0.7, 0, 0);
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
        Coordinate rings = new Coordinate(47, 34);
        robot.intake(1);
        robot.goTo(rings, 1, 0, 0.6);


        robot.launcher.setOnlyFlyWheel(0.8);
        robot.goTo(new Coordinate(71, Robot.hiGoal.y), 0.4, 0, 0.6);
        sleep(800);
        robot.intake(0);
        robot.launcher.magazineShoot();
        robot.launcher.setFlyWheel(0);
    }
}
