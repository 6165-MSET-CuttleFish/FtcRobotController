package org.firstinspires.ftc.teamcode.modules.deposit;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Module;
/**
 * @author Srey Das Sarma
 */
public class Platform extends Module<Platform.State> {
    enum State {
        TRANSIT_IN (0,0),
        IN(0.2,1),
        OUT(0.5,1);
        final double angle;
        final double time;
        State(double angle,double time) {
            this.angle = angle;
            this.time = time;
        }
    }
    StateMachine<State> stateMachine;
    Servo platformL;
    Servo platformR;
    ElapsedTime elapsedTime;
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
        if(stateMachine.getState()==State.TRANSIT_IN){
            setState(State.IN);
        }else if(stateMachine.getState()==State.IN){
            setState(State.OUT);
        }else if(stateMachine.getState()==State.OUT){
            setState(State.TRANSIT_IN);
        }
        elapsedTime.startTime();
        platformL.setPosition(state.getState().angle);
        platformR.setPosition(state.getState().angle);
        elapsedTime.reset();
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {
        platformL = hardwareMap.servo.get("platformLeft");
        platformR = hardwareMap.servo.get("platformRight");
        platformR.setDirection(Servo.Direction.REVERSE);
        setState(State.TRANSIT_IN);
        platformL.setPosition(state.getState().angle);
        platformR.setPosition(state.getState().angle);
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
        /**Conditions:
         * elapsed time passes set time before module reaches position
         */
        if(platformL.getPosition()!= getState().angle&&elapsedTime.time()>getState().time){
            return true;
        }else if(platformL.getPosition()!=(1-platformR.getPosition())){
            return true;
        }
        return false;
    }
}
