package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TesterTeleOp", group = "LinearOpMode")
public class TeleOp extends LinearOpMode {
    Robot robot;
    boolean ninja, armUp, flapUp;
    double lastTime = System.currentTimeMillis();
    double multiplier = 1;
    public final double COUNTS_PER_INCH = 3072;
    Coordinate shootingPosition;
    double shootingAngle;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 18, 18);
        robot.init();
        waitForStart();
        while(opModeIsActive()){
            //setMultiplier();
            robot.setMovement(gamepad1.left_stick_x * multiplier, gamepad1.left_stick_y * multiplier, gamepad1.right_stick_x * multiplier);
//            wobble();
//            robot.intake(-gamepad2.right_stick_y);
//            shooter();
//            telemetry.addData("X Position", robot.position.getX());
//            telemetry.addData("Y Position", robot.position.getY());
//            telemetry.addData("Orientation (Degrees)", robot.position.returnOrientation());
//            telemetry.update();
        }
        Robot.position.stop();
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
    private int i = 0;
    private void autoPowerShots(){
        robot.turnTo(Robot.pwrShots[i], 0.24);
        i++;
        if(i == 4){
            i = 0;
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
            storeCoordinate();
        }
        if(gamepad2.left_trigger >=0.1){
            robot.launcher.setFlyWheel(1);
        }
        else{
            robot.launcher.setFlyWheel(0);

        }
        if(gamepad2.right_trigger >= 0.1){
            robot.launcher.magazineShoot();
            storeCoordinate();
        }
    }
    private void storeCoordinate(){
        shootingPosition = Robot.position.toPoint();
        shootingAngle = Robot.position.radians();
    }
    private void goToLaunchZone(){
        robot.goTo(shootingPosition, 0.5, shootingAngle, 0.5);
    }
}
