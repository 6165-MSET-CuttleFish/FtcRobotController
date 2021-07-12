package org.firstinspires.ftc.teamcode.Components.localizer;
import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.io.File;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal")
public class T265 {

    // Electronics
    public static T265Camera t265Cam;

    // Constants
    public final double ODOMETRY_COVARIANCE = 0.5;
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

    public T265(HardwareMap hardwareMap, double startX, double startY, double startTheta) {
        File file = new File(mapPath);
        if (!file.exists() || file.length() == 0) {
            isEmpty = true;
        }
        if (t265Cam == null) {
            if (!isEmpty) {
                t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, mapPath, hardwareMap.appContext);
            } else {
                t265Cam = new T265Camera(new Transform2d(), ODOMETRY_COVARIANCE, hardwareMap.appContext);
            }
        }
        setCameraPose(startX, startY, startTheta);
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
        x -= -xOffset * Math.sin(theta) - yOffset * Math.cos(theta);
        y -= xOffset * Math.cos(theta) - yOffset * Math.sin(theta);

        t265Cam.setPose(new Pose2d(y * INCH_TO_METER, -x * INCH_TO_METER, new Rotation2d(theta)));
    }

    public void sendOdometryData(double vx, double vy, double theta, double w) {
        double r = Math.hypot(xOffset, yOffset);
        theta += Math.atan2(yOffset, xOffset) - PI/2;
        t265Cam.sendOdometry(vy + r * w * Math.sin(theta), -vx - r * w * Math.cos(theta));
    }

    public void updateCamPose() {
        T265Camera.CameraUpdate state = t265Cam.getLastReceivedCameraUpdate();

        Translation2d translation = new Translation2d(state.pose.getTranslation().getX() / INCH_TO_METER, state.pose.getTranslation().getY() / INCH_TO_METER);
        Rotation2d rotation = state.pose.getRotation();

        chassisSpeeds = state.velocity;

        x = -translation.getY() * Math.sin(theta) - yOffset * Math.cos(theta);
        y = translation.getX() + xOffset * Math.cos(theta) - yOffset * Math.sin(theta);
        theta = MathFunctions.AngleWrap(rotation.getRadians() + Math.toRadians(180));
        if (state.confidence == T265Camera.PoseConfidence.High) {
            confidence = 3;
        } else if (state.confidence == T265Camera.PoseConfidence.Medium) {
            confidence = 2;
        } else if (state.confidence == T265Camera.PoseConfidence.Low) {
            confidence = 1;
        } else {
            confidence = 0;
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