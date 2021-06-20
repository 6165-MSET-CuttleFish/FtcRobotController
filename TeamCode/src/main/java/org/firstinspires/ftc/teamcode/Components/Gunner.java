package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gunner {
    private final StateMachine tripleShot;
    private final StateMachine singleShot;
    private final ElapsedTime externalTimer = new ElapsedTime();
    private static double gunTime = 85.0/1000.0;
    private Servo gunner;
    enum State {
        TRIGGER,
        IN,
        PULLOUT,
        OUT,
        IDLE
    }
    public Gunner(HardwareMap hardwareMap){
        gunner = hardwareMap.servo.get("gunner");
        StateMachineBuilder<State> tripleShotBuilder = new StateMachineBuilder<State>();
        for(int i = 0; i < 3; i ++) {
                tripleShotBuilder = tripleShotBuilder
                        .state(State.TRIGGER)
                        .transitionTimed(gunTime)
                        .onEnter(()->{
                                externalTimer.reset();
                                in();
                        })
                        .state(State.IN)
                        .onEnter(externalTimer::reset)
                        .state(State.PULLOUT)
                        .transitionTimed(gunTime)
                        .onEnter(()->{
                            externalTimer.reset();
                            out();
                        })
                .state(State.OUT);
        }
        tripleShotBuilder = tripleShotBuilder.exit(State.IDLE);
        tripleShot = tripleShotBuilder.build();
        singleShot = new StateMachineBuilder<State>()
                .state(State.TRIGGER)
                .transitionTimed(gunTime)
                .onEnter(externalTimer::reset)
                .state(State.IN)
                .onEnter(externalTimer::reset)
                .state(State.PULLOUT)
                .transitionTimed(gunTime)
                .onEnter(externalTimer::reset)
                .exit(State.IDLE)
                .build();
    }
    public void tripleShot(){
        tripleShot.start();
    }
    public void shoot(){
        singleShot.start();
    }
    public void update(){
        tripleShot.update();
    }
    public State getState(){
        if(tripleShot.getRunning()) return (State) tripleShot.getState();
        return (State) singleShot.getState();
    }
    private void in(){
        gunner.setPosition(0.34);
    }
    private void out(){
        gunner.setPosition(0.48);
    }
}
