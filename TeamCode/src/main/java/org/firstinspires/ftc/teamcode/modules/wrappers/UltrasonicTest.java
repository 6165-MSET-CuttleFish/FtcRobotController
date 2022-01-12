package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UltrasonicTest extends LinearOpMode {
    private UltrasonicDistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {
        distance = new UltrasonicDistanceSensor(hardwareMap, "distance");
        telemetry.addData("init", "done");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Distance", distance.getDistance());
            telemetry.addData("Voltage", distance.analogInput.getMaxVoltage());
            telemetry.update();
        }
    }
}
