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
        robot.unlockIntake();
        robot.launcher.findAngle(robot.hiGoal, robot.position);
        //robot.launcher.setFlyWheel(1);
        sleep(500);
        //robot.launcher.magazineShoot();
        //robot.lockIntake();
        if(robot.discs == 4){
            case4();
        }
        else if(robot.discs == 1){
            case1();
        }
        else{
            case0();
        }
        for(int i = 0; i < Robot.pwrShotLocals.length; i++){
            robot.launcher.setFlyWheel(1);
            robot.goTo(Robot.pwrShotLocals[i], 0.5, 0, 0.7);
            //robot.launcher.aimAt(Robot.pwrShots[i], Robot.position);
            robot.launcher.flapUp();
            robot.launcher.singleRound();
        }
        robot.launcher.setFlyWheel(0);
        Coordinate homePos = new Coordinate(80, robot.position.y);
        robot.goTo(homePos, 0.5, 0, 0.5);
        robot.position.stop();
    }
    public void case0(){
        robot.goTo(Robot.A, 0.5, 0, 1);
    }
    public void case1(){
        robot.goTo(Robot.B, 0.5, 0, 1);
    }
    public void case4(){
        robot.goTo(Robot.C, 0.5, 180, 0.5);
    }
    public void getMoreRings(){
        Coordinate rings = new Coordinate(47, 34);
        robot.intake(1);
        robot.goTo(rings, 0.5, Robot.position.angleTo(rings), 0.8);
        robot.launcher.setFlyWheel(1);
        sleep(1000);
        robot.launcher.magazineShoot();
        robot.launcher.setFlyWheel(0);
    }
}
