package org.firstinspires.ftc.teamcode.Components;

import java.util.concurrent.Callable;

public class StateMachine implements Runnable{
    Callable<Boolean> trigger;
    Runnable action;
    public void setState(Callable<Boolean> trigger, Runnable action){
        this.trigger = trigger;
        this.action = action;
    }
    public void run(){
        try {
            if (trigger.call()) {
                action.run();
            }
            Thread.sleep(20);
        } catch(Exception e){
            e.printStackTrace();
        }

    }
}
