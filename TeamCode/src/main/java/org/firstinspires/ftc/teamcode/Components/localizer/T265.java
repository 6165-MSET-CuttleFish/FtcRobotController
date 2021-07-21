package org.firstinspires.ftc.teamcode.Components.localizer;
import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.util.List;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal")
public class T265 {

    // Electronics
    private static T265Camera t265Cam;
    private final StandardTrackingWheelLocalizer odo;

    // Constants
    public final double ODOMETRY_COVARIANCE = 0.2;
    private final double INCH_TO_METER = 0.0254;
    private final double xOffset = -8.875;
    private final double yOffset = 0.5;

    // State Variables
    private double x, y, theta;
    public int confidence = 0;

    @SuppressLint("SdCardPath")
    private final String mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin";
    public boolean isEmpty = false;
    private boolean exportingMap = true;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public T265(HardwareMap hardwareMap) {
        File file = new File(mapPath);
        if (!file.exists() || file.length() == 0) {
            isEmpty = true;
        }
        if (t265Cam == null) {
            if (!isEmpty) {
                t265Cam = new T265Camera(new Transform2d(new Translation2d(-8.875 * INCH_TO_METER, 0.5 * INCH_TO_METER), new Rotation2d(Math.toRadians(0))), ODOMETRY_COVARIANCE, mapPath, hardwareMap.appContext);
            } else {
                t265Cam = new T265Camera(new Transform2d(new Translation2d(-8.875 * INCH_TO_METER, 0.5 * INCH_TO_METER), new Rotation2d(Math.toRadians(0))), ODOMETRY_COVARIANCE, hardwareMap.appContext);
            }
        }
        odo = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    public void startCam() {
        t265Cam.start();
    }

    public void exportMap() {
        if (exportingMap) {
            exportingMap = false;
            t265Cam.exportRelocalizationMap(mapPath);
        }
    }

    public void stopCam() {
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
        t265Cam.setPose(new Pose2d(x * INCH_TO_METER, y * INCH_TO_METER, new Rotation2d(theta)));
    }

    public void updateCamPose() {
        odo.update();
        T265Camera.CameraUpdate state = t265Cam.getLastReceivedCameraUpdate();
        Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
        Rotation2d rotation = state.pose.getRotation();

        chassisSpeeds = state.velocity;

        x = -translation.getX(); //* Math.sin(theta) - yOffset * Math.cos(theta);
        y = translation.getY(); //+ xOffset * Math.cos(theta) - yOffset * Math.sin(theta);
        theta = rotation.getRadians();
        com.acmerobotics.roadrunner.geometry.Pose2d currVelo = odo.getPoseVelocity();
        if(currVelo != null) {
            t265Cam.sendOdometry(currVelo.getX() * INCH_TO_METER, currVelo.getY() * INCH_TO_METER);
        }
        com.acmerobotics.roadrunner.geometry.Pose2d currPose = odo.getPoseEstimate();
        odo.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(currPose.getX(), currPose.getY(), rotation.getRadians()));
    }

    public com.acmerobotics.roadrunner.geometry.Pose2d getPose() {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(x, y, theta);
    }

    public boolean isStarted() {
        return t265Cam.isStarted();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }
}