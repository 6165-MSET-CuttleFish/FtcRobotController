package org.firstinspires.ftc.teamcode.modules.carousel;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.field.Context;

@TeleOp
public class CarouselTest extends LinearOpMode {
    Carousel carousel;

    @Override
    public void runOpMode() throws InterruptedException {
        carousel = new Carousel(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            carousel.update();
            if (gamepad1.a) {
                carousel.on();
            }
            if (gamepad1.b) {
                carousel.off();
            }
            Context.packet = new TelemetryPacket();
        }
    }
}
