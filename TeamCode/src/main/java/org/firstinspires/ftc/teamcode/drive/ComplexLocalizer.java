package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class ComplexLocalizer implements Localizer {
    StandardTrackingWheelLocalizer odo;
    Robot drive;
    double headingOffset;
    ElapsedTime timer = new ElapsedTime();

    public ComplexLocalizer(HardwareMap hardwareMap, Robot drive) {
        this.drive = drive;
        odo = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return odo.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        odo.setPoseEstimate(pose2d);
        headingOffset = pose2d.getHeading() - drive.getExternalHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return odo.getPoseVelocity();
    }

    @Override
    public void update() {
        odo.update();
//        if (timer.seconds() > 3 && getPoseVelocity().getHeading() < Math.toRadians(40) && getPoseVelocity().getX() < 20 && getPoseVelocity().getY() < 20) {
//            timer.reset();
//            odo.setPoseEstimate(new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(), drive.getExternalHeading() + headingOffset));
//        }
    }
}
