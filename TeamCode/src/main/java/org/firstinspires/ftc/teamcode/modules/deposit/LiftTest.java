package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.controllers.TuningController;

@TeleOp
public class LiftTest extends OpMode {
    Deposit deposit;
    TuningController<Deposit.State> tuningController = new TuningController<>(Deposit.State.values(), 2);
    @Override
    public void init() {
        deposit = new Deposit(hardwareMap, new Intake(hardwareMap));
        Platform.isLoaded = true;
    }

    @Override
    public void loop() {
        deposit.update();
        deposit.setState(tuningController.update());
    }
}
