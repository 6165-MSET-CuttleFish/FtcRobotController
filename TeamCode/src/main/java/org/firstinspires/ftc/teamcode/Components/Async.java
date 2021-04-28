package org.firstinspires.ftc.teamcode.Components;

public class Async {
    public static void start(Runnable block){
        new Thread(block).start();
    }
}
