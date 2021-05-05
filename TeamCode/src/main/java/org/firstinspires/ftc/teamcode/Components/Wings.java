package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.noahbres.jotai.StateMachine;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wings extends SubsystemBase {
    Servo left, right;
    State wingState = State.IDLE;
    enum State {
        ALL_OUT,
        ALL_IN,
        LEFT,
        RIGHT,
        MID,
        VERTICAL,
        MIDLEFT,
        MIDRIGHT,
        UNLOCK,
        IDLE
    }
    public Wings(HardwareMap hardwareMap){
        left = hardwareMap.get(Servo.class,"wallL");
        right = hardwareMap.get(Servo.class,"wallR");
    }
    public State getState(){
        return wingState;
    }
    public void allOut() {
        left.setPosition(0.96);
        right.setPosition(0.18);
        wingState = State.ALL_OUT;
    }
    public void allIn() {
        left.setPosition(.3);
        right.setPosition(.84);
        wingState = State.ALL_IN;
    }
    public void wingsMid() {
        left.setPosition(.85);
        right.setPosition(0.3);
        wingState = State.MID;
    }
    public void vert(){
        left.setPosition(0.6);
        right.setPosition(0.6);
        wingState = State.VERTICAL;
    }
    public void leftOut() {
        left.setPosition(.96);
        right.setPosition(0.84);
        wingState = State.LEFT;
    }
    public void safeLeftOut(){
        left.setPosition(.75);
        right.setPosition(.64);
        wingState = State.MIDLEFT;
    }
    public void rightOut() {
        left.setPosition(.3);
        right.setPosition(0.18);
        wingState = State.RIGHT;
    }
    public void unlockIntake(){
        right.setPosition(0.4);
        wingState = State.UNLOCK;
    }
}
