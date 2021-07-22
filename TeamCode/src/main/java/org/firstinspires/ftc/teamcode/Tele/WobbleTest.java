package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.WobbleArm;
@TeleOp(name = "WobbleTest", group = "Test")
public class WobbleTest extends LinearOpMode {
    WobbleArm wobble;
    @Override
    public void runOpMode() throws InterruptedException {
        wobble = new WobbleArm(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            wobble.update();
            if (gamepad1.a) {
                wobble.claw.grab();
            } else if (gamepad1.b) {
                wobble.claw.release();
            } else if (gamepad1.x) {
                wobble.setState(WobbleArm.State.DOWN);
            } else if (gamepad1.y) {
                wobble.setState(WobbleArm.State.UP);
            }
        }
    }
}
