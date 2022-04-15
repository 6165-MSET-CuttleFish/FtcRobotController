package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.util.controllers.MovingMedian;

@TeleOp
public class DistanceSensorTest extends ModuleTest {
    DistanceSensor frontDist, sideDist;
    MovingMedian movingMedianFront, movingMedianSide;
    @Override
    public void initialize() {
        frontDist = new MB1242(hardwareMap,"rightFrontDS");
        sideDist = new MB1643(hardwareMap, "leftDS");
        ((MB1242) frontDist).setRunDelayMs(100);
        movingMedianFront = new MovingMedian(10);
        movingMedianSide = new MovingMedian(10);
//        ((MB1242) frontDist).ena();
    }

    public void update() {
        double rawFront = frontDist.getDistance(DistanceUnit.INCH);
        double rawRight = sideDist.getDistance(DistanceUnit.INCH);
        double filterFront = movingMedianFront.update(rawFront);
        double filterSide = movingMedianSide.update(rawRight);
        telemetry.addData("RAW Front Distance", rawFront);
        telemetry.addData("RAW Right Distance", rawRight);
        telemetry.addData("FILTER Front Distance", filterFront);
        telemetry.addData("FILTER Right Distance", filterSide);
        telemetry.update();
    }
}
