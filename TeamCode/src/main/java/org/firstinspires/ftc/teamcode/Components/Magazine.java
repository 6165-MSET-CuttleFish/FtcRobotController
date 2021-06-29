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
    StateMachine liftMag;
    StateMachine lowerMag;
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
        liftMag = new StateMachineBuilder<State>()
                .state(State.MOVING_UP)
                .onEnter(()->{
                    externalTimer.reset();
                    up();
                })
                .transitionTimed(0.4)
                .exit(State.UP)
                .build();
        lowerMag = new StateMachineBuilder<State>()
                .state(State.MOVING_DOWN)
                .onEnter(()->{
                    externalTimer.reset();
                    down();
                })
                .transitionTimed(0.4)
                .exit(State.DOWN)
                .build();
    }
    private void up(){
        magLeft1.setPosition(0.75);
        magLeft2.setPosition(0.75);

        magRight1.setPosition(0.23);
        magRight2.setPosition(0.23);
    }
    private void down(){
        magLeft1.setPosition(0.97);
        magLeft2.setPosition(0.97);

        magRight1.setPosition(0.01);
        magRight2.setPosition(0.01);
    }
    public State getState(){
       if(!liftMag.getRunning()) return (State) liftMag.getState();
        return (State) lowerMag.getState();
    }
    public void liftMag(){
        liftMag.start();
    }
    public void lowerMag(){
        lowerMag.start();
    }
    public void update(){
        liftMag.update();
        lowerMag.update();
    }
}
