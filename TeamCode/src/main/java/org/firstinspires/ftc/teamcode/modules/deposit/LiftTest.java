package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;
import org.firstinspires.ftc.teamcode.util.controllers.TuningController;

@TeleOp
public class LiftTest extends ModuleTest {
    Deposit deposit;
    TuningController<Deposit.State> tuningController = new TuningController<>(Deposit.State.values(), 2);
    @Override
    public void init() {
        deposit = new Deposit(hardwareMap, new Intake(hardwareMap));
        Platform.isLoaded = true;
        init(deposit);
    }

    @Override
    public void loop() {
        update();
        deposit.setState(tuningController.update());
    }
}
