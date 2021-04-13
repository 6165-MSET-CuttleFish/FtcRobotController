package org.firstinspires.ftc.teamcode.Components;

import java.util.concurrent.Callable;

public class Async {
    public static void set(Callable<Boolean> condition, Runnable block){
        Thread async = new Thread(()-> {
            while(true){
                try {
                    if (condition.call()) break;
                } catch (Exception exception) {
                    exception.printStackTrace();
                }
            }
            block.run();
        });
        async.start();
    }
    public static void start(Runnable block){
        new Thread(block).start();
    }
}
