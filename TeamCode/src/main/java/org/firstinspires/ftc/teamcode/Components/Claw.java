package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    public Servo grabber, grabber2;
    public State state;
    public enum State{
        GRIP,
        RELEASE
    }
    public Claw(HardwareMap hardwareMap){
        grabber = hardwareMap.servo.get("grabber");
        grabber2 = hardwareMap.servo.get("grabber2");
    }
    public void grab(){

    }
    public void release(){

    }
    public State getState(){
        return state;
    }
}
