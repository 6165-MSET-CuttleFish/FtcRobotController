package org.firstinspires.ftc.teamcode.modules.relocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.Side;

@TeleOp
public class RelocalizationTest extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
        Context.side = Side.CYCLING;
        Context.alliance = Alliance.BLUE;
    }

    @Override
    public void loop() {
        robot.update();
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        0,
                        -gamepad1.right_stick_x
                )
        );
        if (gamepad1.a) robot.rawCorrectPosition();
    }
}
