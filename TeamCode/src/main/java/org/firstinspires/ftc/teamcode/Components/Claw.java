package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    public Servo grabber;
    public State state;
    public enum State{
        GRIP,
        RELEASE
    }
    public Claw(HardwareMap hardwareMap){
        grabber = hardwareMap.servo.get("grabber");
    }
    public void grab(){
        state = State.GRIP;
        grabber.setPosition(0.13);
    }
    public void release(){
        state = State.RELEASE;
        grabber.setPosition(0.63);
    }
    public State getState(){
        return state;
    }
}
