package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Launcher {
    static final double V = 8.85; // 354 in/s
    static final double g = -9.81;

    static double launcherHeight = 0.2032;

    public static void findAngle(double d, targets t/*, Servo flap*/){
        double goalHeight = 0;
        if(t == targets.highGoal) goalHeight = 35.5;
        else if(t == targets.lowGoal) goalHeight = 17;
        else if(t == targets.pwrShot) goalHeight = 23.5;
        goalHeight = inchesToMeters(goalHeight);
        double theta;
        double h = goalHeight - launcherHeight;
        theta = Math.toDegrees((Math.acos((g*d*d/(V*V) - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2);
        System.out.println(theta);
        //flap.setPosition(setAngle(theta));
    }

    public static double setAngle(double theta){
        theta -= 25;
        return -0.1*theta*theta + 3*theta;
    }

    public static void main(String[] args){
        findAngle(2, targets.highGoal);
    }
    enum targets{
        highGoal,
        lowGoal,
        pwrShot
    }

}


