package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Gunner {
    private final StateMachine tripleShot;
    private static double gunTime = 85.0/1000.0;
    private int shotRounds = 0;
    private int targetRounds = 1;
    private final Servo gunner;
    public enum State {
        TRIGGER,
        IN,
        PULLOUT,
        OUT,
        IDLE
    }
    public Gunner(HardwareMap hardwareMap){
        gunner = hardwareMap.servo.get("spanker");
        tripleShot = new StateMachineBuilder<State>()
                .state(State.IDLE)
                .transitionTimed(0)

                .state(State.TRIGGER)
                .transitionTimed(gunTime)
                .onEnter(() -> {
                    this.in();
                    shotRounds++;
                })

                .state(State.IN)
                .transitionTimed(0)

                .state(State.PULLOUT)
                .transitionTimed(gunTime)
                .onEnter(this::out)

                .exit(State.IDLE)
                .build();
    }
    public void tripleShot(int rounds){
        if(!tripleShot.getRunning()) {
            shotRounds = 0;
            targetRounds = rounds;
            tripleShot.setLooping(true);
            tripleShot.start();
        }
    }
    public void shoot() {
        if(!tripleShot.getRunning()) {
            tripleShot.setLooping(true);
            tripleShot.start();
        }
    }
    public void update(){
        tripleShot.update();
        if(targetRounds != 0 && shotRounds == targetRounds){
            shotRounds = 0;
            targetRounds = 1;
            tripleShot.setLooping(false);
        }
    }
    public State getState(){
        return (State) tripleShot.getState();
    }
    private void in(){
        gunner.setPosition(0.8);
    }
    private void out(){
        gunner.setPosition(0.91);
    }
}
