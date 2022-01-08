package org.firstinspires.ftc.teamcode.modules.intake;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class IntakeTest extends ModuleTest {
    Intake intake;

    @Override
    public void initialize() {
        intake = new Intake(hardwareMap);
        setModules(intake);
    }

    @Override
    public void update() {
        intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
