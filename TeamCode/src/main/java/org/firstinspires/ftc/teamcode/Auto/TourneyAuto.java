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
        telemetry.addData("orient", robot.position.returnOrientation());
        robot.grab();
        robot.wobbleArmUp();
        waitForStart();

        robot.scan();
        robot.launcher.findAngle(robot.hiGoal, robot.position);
        robot.launcher.setFlyWheel(1);
        robot.launcher.magazineShoot();
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
        sleep(300);
        robot.release();
        sleep(300);
        robot.wobbleArmUp();
        robot.goTo(Robot.C, 0.4, 0, 1);
        robot.position.stop();
    }
    public void case0(){
        robot.goTo(Robot.A, 0.4, 0, 2);
    }
    public void case1(){
        robot.goTo(Robot.B, 0.4, 0, 2);
    }
    public void case4(){
        robot.goTo(Robot.C, 0.4, 0, 2);

    }
}
