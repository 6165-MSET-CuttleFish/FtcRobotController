package org.firstinspires.ftc.teamcode.modules;

import android.telecom.Call;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Details;

@TeleOp
@Disabled
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Module module = new Module(hardwareMap) {
            @Override
            public void update() {

            }

            @Override
            public void init() {

            }

            @Override
            public boolean isDoingWork() {
                return false;
            }
        };
        waitForStart();
        while (opModeIsActive()) {
            module.update();
            Details.packet = new TelemetryPacket();
        }
    }
}
