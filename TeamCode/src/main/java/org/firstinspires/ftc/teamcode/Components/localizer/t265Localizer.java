package org.firstinspires.ftc.teamcode.Components.localizer;

import android.annotation.SuppressLint;

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

import java.util.function.Consumer;

public class t265Localizer implements Localizer {
    T265 cam;
    @SuppressLint("SdCardPath")
    public t265Localizer(HardwareMap hardwareMap) {
        cam = new T265(hardwareMap, 0, 0, 0);
        //odo = new StandardTrackingWheelLocalizer(hardwareMap);
        if(!cam.isStarted()) cam.startCam();
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
        return null;
    }
}
