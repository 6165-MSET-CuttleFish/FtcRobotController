package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.outoftheboxrobotics.neutrinoi2c.MB1242.AsyncMB1242;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    DistanceSensor frontDist, sideDist;
    @Override
    public void initialize() {
        frontDist = hardwareMap.get(AsyncMB1242.class, "leftFrontDS");
        sideDist = new MB1643(hardwareMap, "rightDS");
    }

    public void update() {
        telemetry.addData("Front Distance", frontDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance", sideDist.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
