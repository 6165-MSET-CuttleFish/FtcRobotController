package org.firstinspires.ftc.teamcode.modules.deposit;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.field.Balance;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Context;

import static org.firstinspires.ftc.teamcode.util.field.Context.balance;

@TeleOp
public class DepositTest extends OpMode {
    Intake intake;
    Deposit deposit;
    GamepadEx primary;
    ToggleButtonReader tippedAway, tippedToward;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap, intake);
        primary = new GamepadEx(gamepad1);
        tippedToward = new ToggleButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER);
        tippedAway = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER);
    }


    @Override
    public void loop() {
        assert Context.telemetry != null;
        telemetry.addData("isLoaded", Platform.isLoaded);
        telemetry.addData("balance", balance);
        telemetry.addData("LB", tippedAway.getState());
        telemetry.addData("RB", tippedToward.getState());
        telemetry.update();
        tippedToward.readValue();
        tippedAway.readValue();
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
        if (tippedAway.isDown() && tippedToward.isDown()) {
            balance = Balance.BALANCED;
        } else if (tippedAway.isDown()) {
            balance = Balance.AWAY;
        } else if (tippedToward.isDown()) {
            balance = Balance.TOWARD;
        }
    }
}
