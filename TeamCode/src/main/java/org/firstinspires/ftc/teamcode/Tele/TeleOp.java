package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode implements Runnable{
    Robot robot;
    boolean ninja, armUp, flapUp;
    double lastTime = System.currentTimeMillis();
    double multiplier = 1;
    public final double COUNTS_PER_INCH = 3072;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 18, 18);
        robot.init();
        waitForStart();
        while(opModeIsActive()){
            setMultiplier();
            robot.setMovement(gamepad1.left_stick_x * multiplier, gamepad1.left_stick_y * multiplier, gamepad1.right_stick_x * multiplier);
//            wobble();
//            robot.intake(-gamepad2.right_stick_y);
//            shooter();
//            telemetry.addData("X Position", robot.position.getX());
//            telemetry.addData("Y Position", robot.position.getY());
//            telemetry.addData("Orientation (Degrees)", robot.position.returnOrientation());
//            telemetry.update();

        }
    }
    private void setMultiplier(){
        if(!ninja && gamepad1.left_bumper && System.currentTimeMillis() >= lastTime + 300){
            ninja = true;
            multiplier /= 3;
            lastTime = System.currentTimeMillis();
        }
        else if(ninja && gamepad1.left_bumper && System.currentTimeMillis() >= lastTime + 300){
            ninja = false;
            multiplier *= 3;
            lastTime = System.currentTimeMillis();
        }
        if(gamepad1.right_bumper && System.currentTimeMillis() >= lastTime + 300){
            multiplier = -multiplier;
            lastTime = System.currentTimeMillis();
        }
    }
    private void wobble(){
        if(gamepad2.a && !armUp){
            robot.wobbleArmUp();
            armUp = true;
        }
        else if(gamepad2.a && armUp){
            robot.wobbleArmDown();
            armUp = false;
        }

        if(gamepad2.x && robot.grabber.getPosition()>0.7){
            robot.grab();
        }
        else if(gamepad2.x && robot.grabber.getPosition()<0.7){
            robot.release();
        }
    }
    public void shooter(){
        if(gamepad2.y&& !flapUp){
            robot.launcher.flapUp();
            sleep(50);
            flapUp = true;
        }
        else if(gamepad2.y && flapUp){
            robot.launcher.flapDown();
            sleep(50);
            flapUp = false;
        }

        if(gamepad2.right_bumper){
            robot.launcher.singleRound();
        }
        if(gamepad2.left_trigger >=0.1){
            robot.launcher.setFlyWheel(1);
        }
        else{
            robot.launcher.setFlyWheel(0);

        }
        if(gamepad2.right_trigger >= 0.1){
            robot.launcher.magazineShoot();
        }
    }
    @Override
    public void run(){

    }
}
