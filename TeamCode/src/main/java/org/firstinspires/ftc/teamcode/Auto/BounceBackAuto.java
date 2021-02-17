package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

@Autonomous(name = "newAuto", group = "LinearOpMode")
class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;
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
        robot.unlockIntake();
        robot.goTo(Robot.pwrShotLocals[0], 0.8, 0, 0.4, ()->{
            robot.wingsIn();
            robot.launcher.setFlyWheel(0.4);
            robot.launcher.flapDown();
        });
        for(Coordinate pt : Robot.pwrShots){
            robot.turnTo(pt, 0.23);
            robot.launcher.singleRound();
        }
        sleep(50);
        robot.launcher.setFlyWheel(0);
        /*
        INSERT BOUNCEBACK CODE HERE
         */
        goToTargetPos();
        robot.goTo(Robot.rightWobble, 0.5, 0, 0.5, ()->robot.grab());
        robot.launcher.setFlyWheel(0.5);
        robot.goTo(new Coordinate(52, Robot.hiGoal.y), 0.5, 0, 0.5, ()->robot.grab());
        robot.launcher.shoot(3);
        /*
        next 3 rings here
         */
    }
    private void goToTargetPos(){
        if(robot.discs == 4){
            case4();
        } else if(robot.discs == 1) {
            case1();
        } else {
            case0();
        }
    }
    private void case4(){
        robot.goTo(Robot.newC, 0.8, Math.toRadians(180), 0.4, ()-> {
            robot.wobbleArmDown();
            if(Robot.position.distanceTo(Robot.newC) < 5){
                robot.release();
            }
        });
        sleep(100);
        robot.wobbleArmUp();
    }
    private void case1(){
        robot.goTo(Robot.newB, 0.8, Math.toRadians(180), 0.4, ()-> {
            robot.wobbleArmDown();
            if(Robot.position.distanceTo(Robot.newB) < 5){
                robot.release();
            }
        });
        sleep(100);
        robot.wobbleArmUp();
    }
    private void case0(){
        robot.goTo(Robot.newA, 0.8, Math.toRadians(180), 0.4, ()-> {
            robot.wobbleArmDown();
            if(Robot.position.distanceTo(Robot.newA) < 5){
                robot.release();
            }
        });
        sleep(100);
        robot.wobbleArmUp();
    }
}
