package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ImprovedLinearOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Details.telemetry = telemetry;
    }
}
