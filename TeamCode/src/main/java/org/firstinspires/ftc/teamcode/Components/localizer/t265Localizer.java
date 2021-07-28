package org.firstinspires.ftc.teamcode.Components.localizer;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PoseUtil;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class t265Localizer implements Localizer {
    T265 cam;

    @SuppressLint("SdCardPath")
    public t265Localizer(HardwareMap hardwareMap) {
        cam = new T265(hardwareMap);
        T265.startCam();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return cam.getPose();
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        cam.setCameraPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @Override
    public void update() {
        cam.updateCamPose();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return PoseUtil.toRRPose2d(cam.getPoseVelo());
    }
}
