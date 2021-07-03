package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Magazine {
    Servo magLeft1, magLeft2;
    Servo magRight1, magRight2;
    public Gunner gunner;
    private final ElapsedTime externalTimer = new ElapsedTime();
    StateMachine magMacro;
    public enum State{
        DOWN,
        MOVING_UP,
        MOVING_DOWN,
        UP
    }
    public Magazine(HardwareMap hardwareMap){
        //gunner = new Gunner(hardwareMap);
        magLeft1 = hardwareMap.servo.get("magLeftBottom");
        magLeft2 = hardwareMap.servo.get("magLeftTop");
        magRight1 = hardwareMap.servo.get("magRightBottom");
        magRight2 = hardwareMap.servo.get("magRightTop");
        magMacro = new StateMachineBuilder<State>()
                .state(State.MOVING_UP)
                .onEnter(this::up)
                .transitionTimed(0.8)
                .state(State.MOVING_DOWN)
                .onEnter(this::down)
                .transitionTimed(0.8)
                .exit(State.DOWN)
                .build();
    }
    public void up(){
        magLeft1.setPosition(0.4);
        magLeft2.setPosition(0.4);

        magRight1.setPosition(0.58);
        magRight2.setPosition(0.58);
    }
    public void down(){
        magLeft1.setPosition(0.71);
        magLeft2.setPosition(0.71);

        magRight1.setPosition(0.27);
        magRight2.setPosition(0.27);
    }
    public State getState(){
       return (State) magMacro.getState();
    }
    public void magMacro() {
        magMacro.start();
    }
    public void update(){
        magMacro.update();
    }
}
