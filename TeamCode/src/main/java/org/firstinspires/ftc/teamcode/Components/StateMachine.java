package org.firstinspires.ftc.teamcode.Components;

import java.util.concurrent.Callable;

public class StateMachine implements Runnable{
    Callable<Boolean> trigger;
    Runnable action;
    boolean isActive;
    public StateMachine setState(Callable<Boolean> trigger, Runnable action){
        this.trigger = trigger;
        this.action = action;
        isActive = true;
        return this;
    }
    @Override
    public void run(){
        try {
            while(isActive) {
                if (trigger.call()) {
                    action.run();
                    isActive = false;
                }
                Thread.sleep(20);
            }
        } catch(Exception e){
            e.printStackTrace();
        }

    }
}
