package org.firstinspires.ftc.teamcode.modules.wrappers;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    UltrasonicDistanceSensor distanceSensor;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void initialize() {
        distanceSensor = new UltrasonicDistanceSensor(hardwareMap, "distance");
        timer.startTime();
    }
    private double startTime, endTime = -1;

    public void update() {
        if(gamepad1.a){
            distanceSensor.internalUpdate();
        }

        if(distanceSensor.digitalInput.getState()){
            if(startTime == 0) startTime = timer.milliseconds();

        }
        else if(!distanceSensor.digitalInput.getState()){
            endTime = timer.milliseconds() - startTime;
            startTime = 0;
        }
        telemetry.addData("Distance1", endTime * 330);
        telemetry.addData("Distance", distanceSensor.getDistance());
        telemetry.addData("Return", distanceSensor.digitalInput.getState());
        telemetry.update();
    }
}
