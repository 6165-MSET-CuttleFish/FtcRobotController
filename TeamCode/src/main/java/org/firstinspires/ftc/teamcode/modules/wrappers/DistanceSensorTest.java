package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    UltrasonicDistanceSensor distanceSensor;
    @Override
    public void initialize() {
        distanceSensor = new UltrasonicDistanceSensor(hardwareMap, "trig", "echo");
        setModules(distanceSensor);
    }

    public void update() {
        telemetry.addData("Measured Distance", distanceSensor.getDistance());
        telemetry.addData("State", distanceSensor.getState());
        telemetry.addData("State Microseconds", distanceSensor.getMicros());
        telemetry.update();
    }
}
