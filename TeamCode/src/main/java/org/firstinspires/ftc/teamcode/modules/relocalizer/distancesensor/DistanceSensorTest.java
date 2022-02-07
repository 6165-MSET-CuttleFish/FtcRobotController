package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    UltrasonicDistanceSensor distanceSensor;
    @Override
    public void initialize() {
        distanceSensor = new UltrasonicDistanceSensor(hardwareMap, "ultra", new Pose2d());
    }

    public void update() {
        telemetry.addData("Measured Distance", distanceSensor.getDistance());
        telemetry.update();
    }
}
