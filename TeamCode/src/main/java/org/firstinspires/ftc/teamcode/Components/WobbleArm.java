package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends Component {
    State state = State.UP;
    public Servo arm1, arm2;
    public Claw claw;
    StateMachine wobbleDropMacro;
    StateMachine wobblePickupMacro;
    public enum State{
        DOWN,
        UP,
        MID,
        TRANSIT,
    }
    public WobbleArm(HardwareMap hardwareMap){
        claw = new Claw(hardwareMap);
        arm1 = hardwareMap.servo.get("wobbleArm1");
        arm2 = hardwareMap.servo.get("wobbleArm2");
        wobbleDropMacro = new StateMachineBuilder<State>()
                .state(State.UP)
                .transitionTimed(0)

                .state(State.MID)
                .transitionTimed(0.3)
                .onEnter(this::mid)

                .exit(State.UP)
                .onExit(() -> {
                    claw.release();
                    up();
                })
                .build();
        wobblePickupMacro = new StateMachineBuilder<State>()
                .state(State.DOWN)
                .transitionTimed(0.25)
                .onEnter(claw::grab)

                .state(State.MID)
                .transitionTimed(0)
                .onEnter(this::mid)

                .exit(State.DOWN)
                .build();
    }
    public void dropMacro(){
        wobbleDropMacro.start();
    }
    public void pickUp(){
        wobblePickupMacro.start();
    }
    public void up() {
        state = State.UP;
        arm1.setPosition(0);
        arm2.setPosition(1);
    }
    public void down() {
        state = State.DOWN;
        arm1.setPosition(0.6);
        arm2.setPosition(0.4);
    }
    public void mid(){
        state = State.MID;
        arm1.setPosition(0.3);
        arm2.setPosition(0.7);
    }
    public State getState() {
        if(wobbleDropMacro.getRunning() || wobblePickupMacro.getRunning()) return State.TRANSIT;
        return state;
    }
    public void update(){
        wobbleDropMacro.update();
        wobblePickupMacro.update();
    }
}
