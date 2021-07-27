package org.firstinspires.ftc.teamcode.Components.localizer;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    public final double ODOMETRY_COVARIANCE = 0.15;
    private final double INCH_TO_METER = 0.0254;

    // State Variables
    private double x, y, theta;

    @SuppressLint("SdCardPath")
    private final String mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin";
    public boolean isEmpty = false;
    private boolean exportingMap = true;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    Transform2d offsets = new Transform2d(new Translation2d(0, 0), new Rotation2d(0));

    public T265(HardwareMap hardwareMap) {
        File file = new File(mapPath);
        if (!file.exists() || file.length() == 0) {
            isEmpty = true;
        }
        if (t265Cam == null) {
            if (!isEmpty) {
                t265Cam = new T265Camera(new Transform2d(new Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), new Rotation2d(Math.toRadians(0))), ODOMETRY_COVARIANCE, mapPath, hardwareMap.appContext);
            } else {
                t265Cam = new T265Camera(new Transform2d(new Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), new Rotation2d(Math.toRadians(0))), ODOMETRY_COVARIANCE, hardwareMap.appContext);
            }
        }
        //t265Cam = new T265Camera(new Transform2d(new Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), new Rotation2d(Math.toRadians(180))), ODOMETRY_COVARIANCE, hardwareMap.appContext);
        odo = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    public static void startCam() {
        t265Cam.start();
    }

    public void exportMap() {
        if (exportingMap) {
            exportingMap = false;
            t265Cam.exportRelocalizationMap(mapPath);
        }
    }

    public static void stopCam() {
        t265Cam.stop();
    }

    public void setCameraPose(double x, double y, double theta) {
        t265Cam.setPose(new Pose2d(-x * INCH_TO_METER, -y * INCH_TO_METER, new Rotation2d(theta)));
        odo.setPoseEstimate(new com.acmerobotics.roadrunner.geometry.Pose2d(x, y, theta));
    }

    public void updateCamPose() {
        odo.update();
        T265Camera.CameraUpdate state = t265Cam.getLastReceivedCameraUpdate();
        Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
        Rotation2d rotation = state.pose.getRotation();

        chassisSpeeds = state.velocity;

        x = -translation.getX();
        y = -translation.getY();
        theta = rotation.getRadians();
        com.acmerobotics.roadrunner.geometry.Pose2d currVelo = odo.getPoseVelocity();
        if (currVelo != null) {
            try {
                t265Cam.sendOdometry(currVelo.getX() * INCH_TO_METER, currVelo.getY() * INCH_TO_METER);
            } catch (Exception ignored) {

            }
        }
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