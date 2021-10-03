package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

public class Platform extends Module<Platform.State> {
    enum State {
        TRANSIT_IN,
        IN,
        TRANSIT_OUT,
        OUT,
    }
    StateMachine<State> stateMachine;
    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Platform(HardwareMap hardwareMap) {
        super(hardwareMap);
        stateMachine = new StateMachineBuilder<State>()
                .build();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {

    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {

    }

    @Override
    public State getState() {
        return stateMachine.getState();
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        return false;
    }
}
