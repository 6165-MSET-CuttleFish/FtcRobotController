package org.firstinspires.ftc.teamcode.modules.carousel;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class CarouselTest extends ModuleTest {
    Carousel carousel;

    @Override
    public void initialize() {
        carousel = new Carousel(hardwareMap);
        setModules(carousel);
    }

    @Override
    public void update() {
        carousel.setPower(gamepad1.right_stick_y);
    }
}
