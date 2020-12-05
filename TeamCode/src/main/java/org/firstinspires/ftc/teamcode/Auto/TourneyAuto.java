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
    Coordinate leftWob;
    int rounds = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 14, 25, 0,18, 18);
        robot.autoInit();
        sleep(1500);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        robot.scan();
        telemetry.addData("Stack Height", robot.height);
        telemetry.addData("Discs", robot.discs);
        telemetry.update();
        //robot.discs = 4;
        robot.goTo(new Coordinate(Robot.position.x + 35, Robot.position.y - 8.5), 0.7, 0, 0);
        dumpWobble();
        rounds ++;
        if(robot.discs == 4){
            Robot.leftWobble.addY(2.2);
            Robot.leftWobble.addX(2);
            targetPos.addX(-6);
        }
        targetPos.addX(-6);
        robot.unlockIntake();
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
        sleep(350);
        robot.wobbleArmUp();

        if(robot.discs != 0) {
            getMoreRings();
        }
//        if(robot.discs !=4){
//            robot.goTo(new Coordinate(targetPos.x - 5, targetPos.y + 15),0.8, 0, 0.3);
//        }
        dumpWobble();
        Coordinate homePos = new Coordinate(88, robot.position.y);
//        if(robot.discs != 4){
//            homePos.y = robot.position.y + 15;
//        }
        robot.goTo(homePos, 0.7, 0, 0);
        robot.position.stop();
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
    public void dumpWobble(){
        if(robot.discs == 4){
            targetPos = Robot.C;
            newcase4();
        }
        else if(robot.discs == 1){
            targetPos = Robot.B;
            newcase1();
        }
        else{
            targetPos = Robot.A;
            newcase0();
        }
        //robot.wobbleArmUp();
    }
    public void getMoreRings(){
        if(robot.discs == 4){
//            Coordinate rings2 = new Coordinate(47, 31.3);
            Coordinate ringSpots[] = new Coordinate[3];
            ringSpots[0] = new Coordinate(49, 31.3);
            ringSpots[1] = new Coordinate(55, 27.3);
            ringSpots[2] = new Coordinate(59, 19.3);
            robot.intake(1);
            //robot.goTo(rings, 1, 0, 0.6);
            robot.goTo(ringSpots[0], 1, 0, 0.6);
            robot.intake(-1);
            sleep(400);

            robot.goTo(ringSpots[1], 0.7, 0, 0.6);
            robot.intake(1);

            robot.goTo(ringSpots[2], 1, 0, 0.6);
            robot.launcher.setOnlyFlyWheel(1);
            robot.goTo(new Coordinate(67, Robot.hiGoal.y), 0.7, 0, 0.6);
            sleep(1000);
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
        else{
            //Coordinate rings = new Coordinate(44, 32);
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
}
