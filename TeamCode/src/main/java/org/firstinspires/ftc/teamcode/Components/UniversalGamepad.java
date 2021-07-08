package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;
import java.util.HashSet;

public class UniversalGamepad implements Component{
    public GamepadEx g1;
    public GamepadEx g2;
    HashSet<KeyReader> readers;
    public ToggleButtonReader clawButton, reverseMode, shieldButton, turretButton, wobbleButton;
    public TriggerReader magButton, intakeButton;

    public UniversalGamepad(LinearOpMode op) {
        g1 = new GamepadEx(op.gamepad1);
        g2 = new GamepadEx(op.gamepad2);
        setButtons();
    }

    @Override
    public void update() {
        for (KeyReader reader : readers) {
            reader.readValue();
        }
    }

    private void setButtons() {
        clawButton = new ToggleButtonReader(g2, GamepadKeys.Button.A);
        reverseMode = new ToggleButtonReader(g1, GamepadKeys.Button.RIGHT_BUMPER);
        magButton = new TriggerReader(g1, GamepadKeys.Trigger.LEFT_TRIGGER);
        shieldButton = new ToggleButtonReader(g1, GamepadKeys.Button.DPAD_DOWN);
        turretButton = new ToggleButtonReader(g2, GamepadKeys.Button.LEFT_BUMPER);
        intakeButton = new TriggerReader(g2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        wobbleButton = new ToggleButtonReader(g2, GamepadKeys.Button.B);
        readers = new HashSet<>(Arrays.asList(clawButton, reverseMode, magButton, shieldButton, turretButton, intakeButton, wobbleButton));
    }
}
