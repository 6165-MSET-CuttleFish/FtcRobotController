package org.firstinspires.ftc.teamcode.modules.relocalizer;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.Side;

@TeleOp
public class RelocalizationTest extends ModuleTest {
    Relocalizer relocalizer;
    BNO055IMU imu;
    @Override
    public void initialize() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        relocalizer = new Relocalizer(hardwareMap, imu);
        Context.side = Side.CYCLING;
        Context.alliance = Alliance.BLUE;
    }

    @Override
    public void update() {
        Context.packet.put("Pitch", relocalizer.getPitch());
        Context.packet.put("Tilt", relocalizer.getTilt());
        Canvas canvas = Context.packet.fieldOverlay();
        canvas.setStrokeWidth(1);
        canvas.setStroke("#4CAF50");
        DashboardUtil.drawRobot(Context.packet.fieldOverlay(), Context.robotPose);
        canvas.setStroke("#F04141");
        DashboardUtil.drawRobot(Context.packet.fieldOverlay(), relocalizer.getPoseEstimate());
    }
}
