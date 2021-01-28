package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.PIDController;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group = "LinearOpMode")
public class TeleOp extends LinearOpMode implements Runnable{
    Robot robot;
    boolean ninja, armUp, flapUp;
    double lastTime = System.currentTimeMillis();
    double multiplier = 1;
    public final double COUNTS_PER_INCH = 3072;
    Coordinate shootingPosition;
    double shootingAngle;
    PIDController pidRotate;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 18, 18, () -> opModeIsActive() && gamepadIdle());
        robot.init();
        pidRotate = new PIDController(.07, 0.014, 0.0044);
        Thread driveTrain = new Thread(this);
        waitForStart();
        driveTrain.start();
        while(opModeIsActive()){
            setMultiplier();
            wobble();
            robot.intake(gamepad2.right_stick_y);
            if(gamepad1.y) robot.goTo(shootingPosition, 1, shootingAngle, 0.7);
            shooter();
            idle();
        }
        Robot.position.stop();
    }
    @Override
    public void run() {
        while(opModeIsActive()) {
            robot.setMovement(gamepad1.left_stick_x * multiplier, gamepad1.left_stick_y * multiplier, gamepad1.right_stick_x * multiplier);
            telemetry.addData("X Position", robot.position.getX());
            telemetry.addData("Y Position", robot.position.getY());
            telemetry.addData("Orientation (Degrees)", robot.position.returnOrientation());
            telemetry.update();
        }
    }
    private void setMultiplier(){
        if(!ninja && gamepad1.left_bumper){
            ninja = true;
            multiplier /= 3;
            sleep(300);
        }
        else if(ninja && gamepad1.left_bumper){
            ninja = false;
            multiplier *= 3;
            sleep(300);
        }
        if(gamepad1.right_bumper){
            multiplier = -multiplier;
            sleep(300);
        }
    }
    public void wobble(){
        if(gamepad2.b && !armUp){
            robot.wobbleArmUp();
            armUp = true;
            sleep(300);
        }
        else if(gamepad2.b && armUp){
            robot.wobbleArmDown();
            sleep(300);
        }
        else if(gamepad2.a){
            robot.wobbleArmVertical();
            armUp = false;
        }
        if(gamepad2.x && robot.grabber.getPosition()>0.3){
            robot.grab();
            sleep(300);
        }
        else if(gamepad2.x && robot.grabber.getPosition()<0.3){
            robot.release();
            sleep(300);
        }
    }
    private void autoPowerShots(){
        for(Coordinate pwrShot : Robot.pwrShots){
            robot.turnTo(pwrShot, 0.24);
        }
    }
    public void shooter(){
        if(gamepad2.left_trigger >=0.1){
            robot.launcher.setFlyWheel(1);
            robot.wingsOut();
            robot.launcher.flapUp();
        }
        else if(gamepad2.left_bumper){
            robot.launcher.setFlyWheel(0.7);
            robot.wingsOut();
            robot.launcher.flapDown();
        }
        else {
            robot.launcher.setFlyWheel(0);
            robot.wingsIn();
            robot.launcher.flapDown();
        }
        if(gamepad2.right_bumper){
            robot.launcher.singleRound();
            storeCoordinate();
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
    private boolean gamepadIdle(){
        return gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && opModeIsActive();
    }
    private void goToLaunchZone(){
        robot.goTo(shootingPosition, 0.5, shootingAngle, 0.5);
    }
}
