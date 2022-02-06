package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    AnalogInput distanceSensor;
    @Override
    public void initialize() {
        distanceSensor = hardwareMap.analogInput.get("ultra");
//        setModules(distanceSensor);
    }

    public void update() {
        double Vobserved = distanceSensor.getVoltage();
        double Vcc = distanceSensor.getMaxVoltage();
        double dist =  ((Vobserved / ((Vcc/1024)) * 6)) / 10;
        telemetry.addData("Max Voltage", Vcc);
        telemetry.addData("Voltage", Vobserved);
        telemetry.addData("Measured Distance", dist);
        telemetry.update();
    }
}
