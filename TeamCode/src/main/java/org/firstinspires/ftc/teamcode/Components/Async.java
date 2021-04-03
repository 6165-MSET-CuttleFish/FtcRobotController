package org.firstinspires.ftc.teamcode.Components;

import java.util.concurrent.Callable;

public class Async {
    public static void start(Callable<Boolean> condition, Runnable block){
        Thread async = new Thread(()-> {
            while(true){
                try {
                    if (!!condition.call()) break;
                } catch (Exception exception) {
                    exception.printStackTrace();
                }
            }
            block.run();
        });
        async.start();
    }
}
