package org.firstinspires.ftc.teamcode.modules.relocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.util.Async;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.Side;

@TeleOp
public class RelocalizationTest extends LinearOpMode {
    Robot robot;
    public void initialize() {
        robot = new Robot(this);
        Context.side = Side.CYCLING;
        Context.alliance = Alliance.BLUE;
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        Async.start(() -> {
            while (opModeIsActive()) {
                robot.relocalizer.updatePoseEstimate(Relocalizer.Sensor.FRONT_RIGHT, Relocalizer.Sensor.LEFT);
            }
        });
        while (opModeIsActive()) {
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
}
