package org.firstinspires.ftc.teamcode.modules.wrappers;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    UltrasonicDistanceSensor distanceSensor;
    @Override
    public void initialize() {
        distanceSensor = new UltrasonicDistanceSensor(hardwareMap, "distance");
    }

    @Override
    public void update() {
        double distance = distanceSensor.getDistance();
        if(gamepad1.a){
            telemetry.addData("Distance", distance);
            telemetry.addData("Voltage", distanceSensor.analogInput.getVoltage());
            telemetry.update();
        }

    }
}
