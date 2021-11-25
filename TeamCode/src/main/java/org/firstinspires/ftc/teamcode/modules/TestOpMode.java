package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.field.Details;
import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;

//@TeleOp
@Disabled
//http://192.168.43.1:8080/dash
public class TestOpMode extends ModuleTest {
    Module module;
    @Override
    public void init() {
        module = new Module(hardwareMap, null) {
            @Override
            public void init() {

            }

            @Override
            public void update() {

            }

            @Override
            public boolean isDoingWork() {
                return false;
            }

            @Override
            public boolean isHazardous() {
                return false;
            }
        };

    }

    @Override
    public void loop() {

    }
}
