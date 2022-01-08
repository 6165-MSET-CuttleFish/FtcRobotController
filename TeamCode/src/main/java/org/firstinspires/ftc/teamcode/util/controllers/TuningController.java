package org.firstinspires.ftc.teamcode.util.controllers;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;

public class TuningController<T> {
    StateMachine<T> stateMachine;
    public TuningController(T[] states, double waitTime) {
        StateMachineBuilder<T> builder = new StateMachineBuilder<T>();
        for (T state : states) {
            builder = builder
                    .state(state)
                    .transitionTimed(waitTime);
        }
        stateMachine = builder.exit(states[0]).build();
        stateMachine.setLooping(true);
    }

    public void start() {
        stateMachine.start();
    }

    public T update() {
        stateMachine.update();
        return stateMachine.getState();
    }
}
