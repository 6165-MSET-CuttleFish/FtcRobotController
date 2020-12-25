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
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 9, 25, 0,18, 18);
        robot.autoInit();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        robot.scan();
        telemetry.addData("Stack Height", robot.height);
        telemetry.addData("Discs", robot.discs);
        telemetry.update();
        robot.unlockIntake();
        robot.goTo(new Coordinate(Robot.position.x + 35, Robot.position.y), 0.7, 0, 0);
        if(robot.discs == 4){
            targetPos = Robot.C;
        }
        else if(robot.discs == 1){
            targetPos = Robot.B;
        }
        else{
            targetPos = Robot.A;
        }
        robot.goTo(targetPos, 0.7, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(550);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
        robot.launcher.setFlyWheel(0.8);
        robot.goTo(Robot.pwrShotLocals[1], 0.6,0, 0.3);
        sleep(300);
        for(int i = 0; i < Robot.pwrShots.length; i++){
            robot.launcher.flapDown();
            robot.turnTo(Robot.pwrShots[i], 0.21);
            robot.launcher.singleRound();
        }
        //sleep(5000);

        robot.launcher.setFlyWheel(0);
        //robot.orient(0, 0.5);
        robot.goTo(Robot.leftWobble, 0.55, 0, 0.4);
        robot.goTo(new Coordinate(Robot.position.x - 8, Robot.position.y), 0.23, 0, 0);
        robot.grab();
        sleep(320);
        robot.wobbleArmUp();

        if(robot.discs != 0) {
            getMoreRings();
        }
//        if(robot.discs !=4){
//            robot.goTo(new Coordinate(targetPos.x - 5, targetPos.y + 15),0.8, 0, 0.3);
//        }
        if(robot.discs == 4){
            targetPos = Robot.C;
            case4();
        }
        else if(robot.discs == 1){
            targetPos = Robot.B;
            case1();
        }
        else{
            targetPos = Robot.A;
            case0();
        }
        Coordinate homePos = new Coordinate(88, robot.position.y);
//        if(robot.discs != 4){
//            homePos.y = robot.position.y + 15;
//        }
        robot.goTo(homePos, 1, 0, 0);
        robot.position.stop();
    }
    public void newcase0(){
        robot.goTo(Robot.newA, 0.7, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(550);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
    }
    public void newcase1(){
        robot.goTo(Robot.newB, 0.7, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(550);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
    }
    public void newcase4(){
        robot.goTo(Robot.newC, 0.8, Math.toRadians(90), 0.4);
        robot.wobbleArmDown();
        sleep(550);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 10), 0.8, 0, 0);
    }
    public void case0(){
        robot.goTo(Robot.A, 0.6, 0, 0);
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x + 10, Robot.position.y), 1, 0, 0);
        robot.goTo(new Coordinate(Robot.position.x , Robot.position.y + 10), 1, 0, 0);

        //robot.goTo(new Coordinate(Robot.position.x, Robot.position.y + 15), 0.7, 0, 0);

    }

    public void case1(){
        robot.goTo(Robot.B, 0.7, 0, 0);
        robot.wobbleArmDown();
        sleep(800);
        robot.release();
        sleep(200);
        robot.goTo(new Coordinate(Robot.position.x + 10, Robot.position.y), 1, 0, 0);
        robot.goTo(new Coordinate(Robot.position.x , Robot.position.y + 10), 1, 0, 0);

    }

    public void case4(){
        Coordinate slowDownPt = new Coordinate(Robot.C);
        slowDownPt.addX(-20);
        slowDownPt.addY(10);
        robot.goTo(slowDownPt, 0.7, Math.toRadians(-180), 0.5);
        robot.wobbleArmDown();
        robot.goTo(Robot.C, 0.4, Math.toRadians(-180), 0.4);
        robot.release();
        sleep(500);
        robot.goTo(new Coordinate(Robot.position.x - 10, Robot.position.y), 0.8, 0, 0);
    }
    public void getMoreRings(){
            Coordinate rings2 = new Coordinate(47, 31.3);
            robot.intake(1);
            //robot.goTo(rings, 1, 0, 0.6);
            robot.goTo(rings2, 1, 0, 0.6);
            robot.launcher.setOnlyFlyWheel(1);
            robot.goTo(new Coordinate(67, Robot.hiGoal.y), 0.7, 0, 0.6);
            if(robot.discs == 4){
                sleep(2300);
            }
            else{
                sleep(1000);
            }
            if (robot.discs == 4) {
                robot.intake(-1);
            } else {
                robot.intake(0);
            }
            sleep(100);
            if(robot.discs == 4) {
                robot.launcher.magazineShoot();
            }
            else{
                robot.launcher.singleRound();
            }
            robot.launcher.setFlyWheel(0);
            robot.intake(0);


    }
}
