package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    final double V = 50; // random value rn
    final double g = -9.81;

    public Servo angle;
    public DcMotor launchWheel;
    public double launcherHeight;
    public double goalHeight;
    public double theta;

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


    public void findAngle(double x, double y, double goalX, double goalY){
        double h = goalHeight - launcherHeight;
        double d = Math.hypot(Math.abs(x-goalX), Math.abs(y-goalY));

        double sinPlus = (d+Math.sqrt(d*d + ((4*g*d*d)/(2*V*V)) - 4*h))/-2;
        double sinMinus = (d-Math.sqrt(d*d + ((4*g*d*d)/(2*V*V)) - 4*h))/-2;
        if(sinPlus > 1 || sinPlus < -1){
            theta = Math.asin(sinMinus);
        }
        else if(sinMinus > 1 || sinMinus < -1){
            theta = Math.asin(sinPlus);
        }
        else{
            theta = Math.min(Math.asin(sinMinus), Math.asin(sinPlus));
        }

        angle.setPosition(theta/90);
    }



}
