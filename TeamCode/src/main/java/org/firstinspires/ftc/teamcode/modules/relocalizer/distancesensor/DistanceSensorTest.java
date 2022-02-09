package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    DistanceSensor distanceSensor;
    @Override
    public void initialize() {
        distanceSensor = new MB1242(hardwareMap.i2cDeviceSynch.get("ultra"));
    }

    public void update() {
        telemetry.addData("Measured Distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
