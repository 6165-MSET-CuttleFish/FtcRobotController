package org.firstinspires.ftc.teamcode.Tele;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;

@TeleOp(name = "MagTest", group = "Test")
public class MagTest extends LinearOpMode {
    Magazine magazine;
    Gunner gunner;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        magazine = new Magazine(hardwareMap);
        gunner = new Gunner(hardwareMap);
        intake = new Intake(hardwareMap);
        GamepadEx g2 = new GamepadEx(gamepad2);
        ToggleButtonReader magButton = new ToggleButtonReader(g2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        waitForStart();
        while(opModeIsActive()){
            magazine.update();
            gunner.update();
            intake.update();
            magButton.readValue();
            if(magButton.wasJustPressed()){
                magazine.magMacro();
            }
            intake.setPower(g2.getRightY());
            if (Magazine.currentRings != 0 && magazine.getState() == Magazine.State.DOWN) {
                gunner.shoot();
            }
            telemetry.addData("dist", magazine.getRange());
            telemetry.update();
        }
    }
}
