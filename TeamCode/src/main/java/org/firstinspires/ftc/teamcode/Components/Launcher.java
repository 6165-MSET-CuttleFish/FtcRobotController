package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    final static double V = 50; // random value rn
    final static double g = -9.81;

    public Servo angle;
    public DcMotor launchWheel;
    public static double launcherHeight = 0;
    public static double goalHeight = 10;
    public static double theta;

    HardwareMap map;
    DcMotor.RunMode newRun;

    public Launcher(DcMotor.RunMode runMode, HardwareMap imported, double launcherHeight){
        map = imported;
        newRun = runMode;
        this.launcherHeight = launcherHeight;

        launchWheel = map.dcMotor.get("launchWheel");
        angle = map.servo.get("angle");

        launchWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchWheel.setMode(newRun);
        launchWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public static void findAngle(double x, double y, double goalX, double goalY){
        double h = goalHeight - launcherHeight;
        double d = Math.hypot(Math.abs(x-goalX), Math.abs(y-goalY));
        theta = (Math.acos((g*d*d/V*V - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2;
        System.out.println(theta);
    }

    public static void main(String[] args){
        //findAngle(0, 2, 0, 0);
    }

}
