package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    DistanceSensor frontDist, sideDist;
    @Override
    public void initialize() {
        frontDist = new MB1242(hardwareMap.i2cDeviceSynch.get("rightFrontDS"));
        sideDist = new MB1643(hardwareMap, "leftDS");
    }

    public void update() {
        telemetry.addData("Front Distance", frontDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", sideDist.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
