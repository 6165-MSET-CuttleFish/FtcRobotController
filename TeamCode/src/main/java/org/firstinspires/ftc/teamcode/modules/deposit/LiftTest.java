package org.firstinspires.ftc.teamcode.modules.deposit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.controllers.TuningController;

@TeleOp
@Disabled
public class LiftTest extends ModuleTest {
    Deposit deposit;
    TuningController<Deposit.Level> tuningController = new TuningController<>(Deposit.Level.values(), 2);
    @Override
    public void initialize() {
        deposit = new Deposit(hardwareMap, new Intake(hardwareMap));
        Deposit.isLoaded = true;
        setModules(deposit);
    }

    @Override
    public void opening() {
        tuningController.start();
    }

    @Override
    public void update() {
        deposit.setLevel(tuningController.update());
    }
}
