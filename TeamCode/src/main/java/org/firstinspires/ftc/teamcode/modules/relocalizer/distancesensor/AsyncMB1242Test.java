package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.outoftheboxrobotics.neutrinoi2c.MB1242.AsyncMB1242;

@TeleOp
public class AsyncMB1242Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AsyncMB1242 sensor = hardwareMap.get(AsyncMB1242.class, "leftFrontDS");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}