package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.OpModeType;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.Side;
import org.firstinspires.ftc.teamcode.localizers.T265;

@TeleOp(name = "T265Test", group = "test")
@Disabled
public class t265LocalizationTest extends OpMode {
    Robot robot;

    @Override
    public void stop() {
        T265.stopCam();
        super.stop();
    }

    @Override
    public void init() {
        robot = new Robot(this, OpModeType.TELE, Side.RED);
    }

    @Override
    public void loop() {
        robot.update();
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        Pose2d poseEstimate = robot.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }
}
