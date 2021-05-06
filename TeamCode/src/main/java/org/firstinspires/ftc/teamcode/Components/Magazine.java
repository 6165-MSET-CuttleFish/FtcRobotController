package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Magazine {
    Servo mag;
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
        gunner = new Gunner(hardwareMap);
        mag = hardwareMap.servo.get("mag");
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
        mag.setPosition(0.3);
    }
    private void down(){
        mag.setPosition(0);
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
