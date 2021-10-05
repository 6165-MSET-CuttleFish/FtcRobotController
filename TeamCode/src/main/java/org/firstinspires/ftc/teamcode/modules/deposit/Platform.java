package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
/**
 * @author Srey Das Sarma
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
        if(stateMachine.getState()==State.TRANSIT_IN){
            return true;
        }else if(stateMachine.getState()==State.TRANSIT_OUT){
            return true;
        }
        return false;
    }
}
