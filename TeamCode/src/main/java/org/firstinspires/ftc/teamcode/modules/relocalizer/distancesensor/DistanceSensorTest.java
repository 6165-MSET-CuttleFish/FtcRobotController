package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    UltrasonicDistanceSensor frontDist, sideDist;
    @Override
    public void initialize() {
        frontDist = new UltrasonicDistanceSensor(
                hardwareMap,
                "leftFrontDS",
                UltrasonicDistanceSensor.SensorType.LongRange
        );
        sideDist = new UltrasonicDistanceSensor(
                hardwareMap,
                "rightDS",
                UltrasonicDistanceSensor.SensorType.ShortRange
        );
    }

    public void update() {
        telemetry.addData("Front Distance", frontDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance", sideDist.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
