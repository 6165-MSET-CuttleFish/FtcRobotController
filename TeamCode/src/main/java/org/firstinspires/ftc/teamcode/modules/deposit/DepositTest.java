package org.firstinspires.ftc.teamcode.modules.deposit;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;

@TeleOp
public class DepositTest extends ModuleTest {
    Intake intake;
    Deposit deposit;
    GamepadEx primary;
    ToggleButtonReader tipped;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap, intake);
        primary = new GamepadEx(gamepad1);
        tipped = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER);
        init(intake, deposit);
    }


    @Override
    public void loop() {
        update();
        tipped.readValue();
        intake.setPower(gamepad1.right_trigger);
        if (gamepad1.x) {
            deposit.setState(Deposit.State.LEVEL2);
        }
        if (gamepad1.b) {
            deposit.setState(Deposit.State.LEVEL3);
        }
        if (gamepad1.y) {
            deposit.setState(Deposit.State.IDLE);
        }
        if (gamepad1.a) {
            deposit.dump();
        }
        Platform.isUnbalanced = tipped.getState();
        telemetry.addData("isLoaded", Platform.isLoaded);
        telemetry.addData("isUnbalanced", Platform.isUnbalanced);
    }
}
