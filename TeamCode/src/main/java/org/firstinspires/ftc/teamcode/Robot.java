package org.firstinspires.ftc.teamcode;

import java.lang.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import static org.firstinspires.ftc.teamcode.Odometry.MathFunctions.*;

public class Robot {
    public OdometryGlobalCoordinatePosition position;
    public DcMotor topLeft;
    public DcMotor topRight;
    public DcMotor botLeft;
    public DcMotor botRight;

    public Launcher launcher;
    Orientation orientation = new Orientation();
    Orientation lastAngles = new Orientation();
    PIDController pidRotate, pidDrive, pidStrafe, pidCurve, pidCorrection;
    DcMotor.RunMode newRun;
    HardwareMap map;

    public double movement_x;
    public double movement_y;
    public double movement_turn;

    private boolean auto;
    public double ninja = 1;
    public Robot(DcMotor.RunMode runMode, HardwareMap imported, double x, double y, double robotLength, double robotWidth) {
        auto = true;
        map = imported;
        newRun = runMode;
        topLeft = map.dcMotor.get("topLeft");
        topRight = map.dcMotor.get("topRight");
        botLeft = map.dcMotor.get("botLeft");
        botRight = map.dcMotor.get("botRight");


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(newRun);//set to imported runMode
        topRight.setMode(newRun);
        botLeft.setMode(newRun);
        botRight.setMode(newRun);


        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);//in reverse because of motor direction
        botLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //bot = new OdometryGlobalCoordinatePosition(topLeft, topRight, botLeft, 560, 760, x, y);
        position  = new OdometryGlobalCoordinatePosition(topLeft, topRight, botLeft, 307.699557, 760, x, y);

    }
    public static int index = 0;
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed){

        double distance = Math.hypot(pt.x - position.getX(), pt.y - position.y);
        if(distance < 5) {
            setMovement(0, 0, 0);
            index ++;
            return;
        }
        distance = Math.hypot(pt.x - position.x, pt.y - position.y);
        //double absAngleToTarget = Math.atan2(y - worldAngle_rad, x - worldAngle_rad);
        double absAngleToTarget = Math.atan2(pt.y - position.y, pt.x - position.x);
        //System.out.println("Abs " + absAngleToTarget);
        double relAngleToPoint = AngleWrap(absAngleToTarget - position.robotOrientationRadians + Math.toRadians(90));
        //System.out.println("Rel " + relAngleToPoint);
        double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
        double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
//        double relativeTurnAngle = relAngleToPoint - Math.toRadians(180) + preferredAngle;
//        double movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)*turnSpeed;

        movement_x = movementXPower * power;
        movement_y = movementYPower * power;
        double relTurnAngle = relAngleToPoint - Math.toRadians(90) + preferredAngle;
        movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
        setMovement(movement_x, movement_y, movement_turn);
    }
    public void setMovement(double lx, double ly, double rx){
        movement_x = lx;
        movement_y = ly;
        movement_turn = rx;
        topLeft.setPower(Range.clip(ninja * ly + lx + rx, -1, 1));//1.1   ; -.9
        topRight.setPower(Range.clip(ninja * ly - lx - rx, -1, 1));//-1.1  ; .9
        botLeft.setPower(Range.clip(ninja * ly - lx + rx, -1, 1));//-.9   ; 1.1
        botRight.setPower(Range.clip(ninja * ly + lx - rx, -1, 1));//.9  ; -1.1

    }
    public void init(){
       Thread newThread = new Thread(position);
       newThread.start();
    }
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public double getHeading() {//for overall
        return position.returnOrientation();
    }
    public final void idle() {
        Thread.yield();
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    enum heading{
        forward,
        backward,
        left,
        right
    }
}