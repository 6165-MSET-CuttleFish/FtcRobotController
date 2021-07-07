package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gunner extends Component{
    private final StateMachine shoot;
    private static double gunTime = 85.0/1000.0;
    private int shotRounds = 0;
    private int targetRounds = 1;
    private final Servo gunner;
    public enum State {
        TRIGGER,
        IN,
        PULLOUT,
        OUT,
        IDLE,
    }
    public Gunner(HardwareMap hardwareMap){
        gunner = hardwareMap.servo.get("spanker");
        shoot = new StateMachineBuilder<State>()
                .state(State.OUT)
                .transitionTimed(0)

                .state(State.TRIGGER)
                .transitionTimed(gunTime)
                .onEnter(this::in)

                .state(State.IN)
                .transitionTimed(0)
                .onEnter(() -> shotRounds++)

                .state(State.PULLOUT)
                .transitionTimed(gunTime)
                .onEnter(this::out)

                .exit(State.OUT)
                .build();
    }
    public void shoot(int rounds){
        if(!shoot.getRunning()) {
            shotRounds = 0;
            targetRounds = rounds;
            shoot.setLooping(true);
            shoot.start();
        }
    }
    public void shoot() {
        if(!shoot.getRunning()) {
            shoot.setLooping(true);
            shoot.start();
        }
    }
    public void update(){
        shoot.update();
        if(targetRounds != 0 && shotRounds == targetRounds){
            shotRounds = 0;
            targetRounds = 1;
            shoot.setLooping(false);
        }
    }
    public State getState(){
        return shoot.getRunning() ? (State) shoot.getState() : State.IDLE;
    }
    private void in(){
        gunner.setPosition(0.8);
    }
    private void out(){
        gunner.setPosition(0.91);
    }
}
