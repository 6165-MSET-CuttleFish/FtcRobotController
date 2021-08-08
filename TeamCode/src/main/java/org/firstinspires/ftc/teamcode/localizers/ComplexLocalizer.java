package org.firstinspires.ftc.teamcode.localizers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class ComplexLocalizer implements Localizer {
    ThreeTrackingWheelLocalizer odo, perpOdo;
    t265Localizer t265Localizer;
    double headingOffset;
    ElapsedTime timer = new ElapsedTime();
    HashMap<Localizer, Double> localizers;
    Pose2d pose2d, poseVelo;

    public ComplexLocalizer(HardwareMap hardwareMap) {
        localizers.put(odo = new StandardTrackingWheelLocalizer(hardwareMap), 1.0);
        localizers.put(perpOdo = new PerpTrackingWheelLocalizer(hardwareMap), 1.0);
        localizers.put(t265Localizer = new t265Localizer(hardwareMap), 0.1);
        pose2d = new Pose2d();
        poseVelo = new Pose2d();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return pose2d;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        for (Localizer localizer : localizers.keySet()) {
            localizer.setPoseEstimate(pose2d);
        }
        headingOffset = pose2d.getHeading() - t265Localizer.getPoseEstimate().getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelo;
    }

    @Override
    public void update() {
        Pose2d pose2d = new Pose2d();
        Pose2d poseVelo = new Pose2d();
        for (Localizer localizer : localizers.keySet()) {
            localizer.update();
            pose2d.plus(localizer.getPoseEstimate()).times(localizers.get(localizer));
            poseVelo.plus(localizer.getPoseVelocity()).times(localizers.get(localizer));
            if (localizer instanceof ThreeTrackingWheelLocalizer && timer.seconds() > 2) {
                Pose2d pose =  localizer.getPoseEstimate();
                localizer.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), t265Localizer.getPoseEstimate().getHeading() + headingOffset));
            } else if (timer.seconds() > 2) {
                timer.reset();
            }
        }
        double weight = weight();
        pose2d.div(weight);
        poseVelo.div(weight);
        this.pose2d = pose2d;
        this.poseVelo = poseVelo;
    }

    private double weight() {
        double weight = 0;
        for (double w : localizers.values()) {
            weight += w;
        }
        return weight;
    }
}
