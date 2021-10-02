package org.firstinspires.ftc.teamcode.util;

public class Async {
    public static void start(Runnable block){
        new Thread(block).start();
    }
}
