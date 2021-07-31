package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;

public class WobbleArm implements Component {
    static State state = State.UP;
    public Servo arm1, arm2;
    public Claw claw;
    static StateMachine wobbleDropMacro;
    static StateMachine wobblePickupMacro;
    ElapsedTime timer = new ElapsedTime();
    public enum State{
        MOVING_DOWN,
        DOWN,
        MOVING_UP,
        UP,
        MOVING_MID,
        MID,
        MACRO,
    }
    public WobbleArm(HardwareMap hardwareMap){
        claw = new Claw(hardwareMap);
        arm1 = hardwareMap.servo.get("wobbleArm1");
        arm2 = hardwareMap.servo.get("wobbleArm2");
        if (opModeType == OpModeType.AUTO) {
            state = State.MID;
        } else {
            state = State.UP;
        }
        wobbleDropMacro = new StateMachineBuilder<State>()
                .state(State.UP)
                .transitionTimed(0)

                .state(State.DOWN)
                .transitionTimed(0.3)
                .onEnter(this::drop)

                .state(State.MID)
                .transitionTimed(0.2)
                .onEnter(() -> claw.release())

                .state(State.MACRO)
                .transitionTimed(0.2)
                .onEnter(() -> claw.grab())

                .exit(State.UP)
                .onExit(() -> {
                    up();
                    state = State.UP;
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
                .onExit(() -> state = State.MID)
                .build();
    }
    public void dropMacro(){
        if(!getRunning()) wobbleDropMacro.start();
    }
    public void pickUp(){
        if(!getRunning() && getState() != State.UP) wobblePickupMacro.start();
    }
    private void up() {
        arm1.setPosition(0);
        arm2.setPosition(1);
    }
    private void down() {
        arm1.setPosition(0.6);
        arm2.setPosition(0.4);
    }
    private void drop() {
        arm1.setPosition(0.45);
        arm2.setPosition(0.55);
    }
    private void mid(){
        arm1.setPosition(0.1);
        arm2.setPosition(0.9);
    }
    public static State getState() {
        try {
            if (wobbleDropMacro.getRunning() || wobblePickupMacro.getRunning()) return State.MACRO;
            return state;
        } catch (Exception ignored) {
            return state;
        }
     }
    public void setState(State state) {
        WobbleArm.state = state;
    }
    public void update() {
        if(!getRunning()) {
            switch (state) {
                case UP:
                    up();
                    timer.reset();
                    break;
                case MID:
                    mid();
                    timer.reset();
                    break;
                case DOWN:
                    down();
                    timer.reset();
                    break;
                case MOVING_DOWN:
                    down();
                    if (timer.seconds() > 0.3) {
                        setState(State.DOWN);
                        claw.release();
                    }
                    break;
                case MOVING_UP:
                    up();
                    if (timer.seconds() > 0.3) {
                        setState(State.UP);
                    }
                    break;
                case MOVING_MID:
                    mid();
                    if (timer.seconds() > 0.3) {
                        setState(State.MID);
                    }
                    break;
            }
        }
        wobbleDropMacro.update();
        wobblePickupMacro.update();
    }
    public boolean getRunning() {
        return wobbleDropMacro.getRunning() || wobblePickupMacro.getRunning();
    }
}
