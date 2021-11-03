package org.firstinspires.ftc.teamcode.util;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;

public class TuningController<T> {
    T[] states;
    StateMachine<T> stateMachine;

    public TuningController(T[] states, double waitTime) {
        this.states = states;
        StateMachineBuilder<T> builder = new StateMachineBuilder<T>();
        for (T state : states) {
            builder = builder
                    .state(state)
                    .transitionTimed(waitTime);
        }
        stateMachine = builder.exit(states[0]).build();
        stateMachine.setLooping(true);
    }

    public T update() {
        stateMachine.update();
        return stateMachine.getState();
    }
}
