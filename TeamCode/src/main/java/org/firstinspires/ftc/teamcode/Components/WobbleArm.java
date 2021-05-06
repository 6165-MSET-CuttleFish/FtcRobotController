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
        wobbleDropMacro = new StateMachineBuilder<State>()
                .state(state)
                .exit(State.IN)
                .build();
    }
    public void dropMacro(){
        wobbleDropMacro.start();
    }
    public State getState() {
        if(wobbleDropMacro.getRunning()) return (State) wobbleDropMacro.getState();
        return state;
    }
}
