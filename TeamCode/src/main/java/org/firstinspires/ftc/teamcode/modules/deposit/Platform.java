package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    DcMotorEx platform;
    private Platform state;
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
        if(stateMachine.getState()== State.TRANSIT_IN){
            setState(State.IN);
        }else if(stateMachine.getState()== State.IN){
            setState(State.TRANSIT_OUT);
        }else if(stateMachine.getState()== State.TRANSIT_OUT){
            setState(State.OUT);
        }else if(stateMachine.getState()== State.OUT){
            setState(State.TRANSIT_IN);
        }else{
            isHazardous();
        }
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platform = hardwareMap.get(DcMotorEx.class, "platform");
    }

    @Override
    public State getState() {
        return stateMachine.getState();
    }

    /**
     * Set a new state for the module
     * @param state New state of the module
     */
    public void setState(Platform state) {
        this.state = state;
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        return false;
    }
}
