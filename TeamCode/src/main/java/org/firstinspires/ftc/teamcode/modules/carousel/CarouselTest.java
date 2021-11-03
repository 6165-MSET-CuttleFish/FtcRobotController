package org.firstinspires.ftc.teamcode.modules.carousel;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Details;

@TeleOp
public class CarouselTest extends LinearOpMode {
    Carousel carousel;

    @Override
    public void runOpMode() throws InterruptedException {
        carousel = new Carousel(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            carousel.update();
            // TODO: add driver control for testing module here
            if (gamepad1.b) {
                carousel.setState(Carousel.State.ON);
            }
            if (gamepad1.a) {
                carousel.setState(Carousel.State.IDLE);
            }
            Details.packet = new TelemetryPacket();
        }
    }
}
