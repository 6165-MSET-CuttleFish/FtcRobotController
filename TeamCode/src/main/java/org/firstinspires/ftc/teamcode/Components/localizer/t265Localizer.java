package org.firstinspires.ftc.teamcode.Components.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.util.PoseUtil;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class t265Localizer implements Localizer {
    T265Camera cam;
    public t265Localizer(HardwareMap hardwareMap){
        cam = new T265Camera(new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)), 0.5, hardwareMap.appContext);
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return PoseUtil.toRRPos2d(cam.getLastReceivedCameraUpdate().pose);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        cam.setPose(PoseUtil.toFTCLibPos2d(pose2d));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return cam.getLastReceivedCameraUpdate().velocity;
    }

    @Override
    public void update() {
        cam.sendOdometry();
    }
}
