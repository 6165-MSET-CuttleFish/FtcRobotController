package org.firstinspires.ftc.teamcode.modules.capstone;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Capstone capstone;
    Carousel carousel;
    GamepadEx gamepadEx;
    ButtonReader incrementVertical, decrementVertical, incrementHorizontal, decrementHorizontal;
    @Override
    public void initialize() {
        capstone = new Capstone(hardwareMap);
        carousel = new Carousel(hardwareMap);
        setModules(capstone, carousel);
        gamepadEx = new GamepadEx(gamepad1);
        setKeyReaders(
                incrementHorizontal = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_RIGHT),
                decrementHorizontal = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_LEFT),
                incrementVertical = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_UP),
                decrementVertical = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_DOWN)
        );
    }

    @Override
    public void update() {
        capstone.setTape(gamepad1.right_trigger - gamepad1.left_trigger);
        capstone.setVerticalTurret(gamepad1.left_stick_y);
        capstone.setHorizontalTurret(gamepad1.right_stick_x);
        carousel.setPower(gamepad2.right_stick_y);
        if (incrementHorizontal.wasJustPressed()) {
            capstone.incrementHorizontal(1);
        } else if (decrementHorizontal.wasJustPressed()) {
            capstone.incrementHorizontal(-1);
        }
        if (incrementVertical.wasJustPressed()) {
            capstone.incrementVertical(1);
        } else if (decrementVertical.wasJustPressed()) {
            capstone.incrementVertical(-1);
        }
        telemetry.addData("",capstone.getVerticalTurretPos());
    }
}
