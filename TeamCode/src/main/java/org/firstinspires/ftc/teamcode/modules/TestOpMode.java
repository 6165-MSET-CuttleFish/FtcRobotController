package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Details;

@TeleOp
@Disabled
//http://192.168.43.1:8080/dash
public class TestOpMode extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        Module module = new Module(hardwareMap, null) {
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

            /**
             * @return Whether the module is currently in a hazardous state
             */
            @Override
            public boolean isHazardous() {
                return false;
            }
        };
        waitForStart();
        while (opModeIsActive()) {
            module.update();
            dashboard.sendTelemetryPacket(Details.packet);
            Details.packet = new TelemetryPacket();
        }
    }
}
