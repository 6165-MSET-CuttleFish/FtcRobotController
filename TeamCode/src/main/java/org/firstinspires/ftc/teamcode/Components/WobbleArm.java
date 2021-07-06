package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends SubsystemBase {
    State state = State.IN;
    public Servo arm1, arm2;
    public Claw claw;
    StateMachine wobbleDropMacro;
    public enum State{
        OUT,
        IN,
        MID,
        TRANSIT
    }
    public WobbleArm(HardwareMap hardwareMap){
        claw = new Claw(hardwareMap);
        arm1 = hardwareMap.servo.get("wobbleArm1");
        arm2 = hardwareMap.servo.get("wobbleArm2");
        wobbleDropMacro = new StateMachineBuilder<State>()
                .state(state)
                .exit(State.IN)
                .build();
    }
    public void dropMacro(){
        wobbleDropMacro.start();
    }
    public void up() {
        arm1.setPosition(0);
        arm2.setPosition(1);
    }
    public void down() {
        arm1.setPosition(0.6);
        arm2.setPosition(0.4);
    }
    public void mid(){
        arm1.setPosition(0.4);
        arm2.setPosition(0.6);
    }
    public State getState() {
        if(wobbleDropMacro.getRunning()) return (State) wobbleDropMacro.getState();
        return state;
    }
    public void update(){
        wobbleDropMacro.update();
    }
}
