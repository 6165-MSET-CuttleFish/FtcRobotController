package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;

@TeleOp
public class DepositTest extends ModuleTest {

    Intake intake;
    Deposit deposit;
    Platform platform;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap, intake);
        platform = deposit.platform;
        init(intake, deposit);
    }


    @Override
    public void loop() {
        update();
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
            platform.dump();
        }
    }
}
