package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Mechanism containing the freight and that which rotates outwards to deposit the freight using servos
 * @author Sreyash, Martin
 */
public class Platform extends Module<Platform.State> {
    enum State {
        TRANSIT_IN,
        IN,
        TRANSIT_OUT,
        OUT,
    }
    StateMachine<State> stateMachine;
    Servo platform;
    ElapsedTime timer;
    private Platform state;
    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Platform(HardwareMap hardwareMap) {
        super(hardwareMap);
        stateMachine = new StateMachineBuilder<State>()
                .state(State.IN)
                .onEnter(this::out)
                .transition(() -> true) // () -> is anonymous function that returns true

                .state(State.TRANSIT_OUT)
                .onEnter(this::out)
                .transitionTimed(0.5)

                .state(State.OUT)
                .transition(() -> true)

                .state(State.TRANSIT_IN)


                .state(State.IN)

                .build();
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platform = hardwareMap.get(Servo.class, "platform");
    }

    @Override
    public State getState() {
        return stateMachine.getState();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {
        switch (getState()) {

        }
        if(stateMachine.getState()== State.TRANSIT_IN){
            setState(State.IN);
        }else if(stateMachine.getState()== State.IN){
            setState(State.TRANSIT_OUT);
        }else if(stateMachine.getState()== State.TRANSIT_OUT){
            setState(State.OUT);
        }else if(stateMachine.getState()== State.OUT){
            setState(State.TRANSIT_IN);
        }else{
            isDoingWork();
        }
    }

    /**
     * Extends the platform out
     */
    private void out() {
        platform.setPosition(1);
    }

    /**
     * Return platform to rest
     */
    private void in() {
        platform.setPosition(0);
    }


    /**
     * Set a new state for the module
     * @param state New state of the module
     */
    public void setState(Platform state) {
        this.state = state;
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    @Override
    public boolean isDoingWork() {
        return false;
    }
}
