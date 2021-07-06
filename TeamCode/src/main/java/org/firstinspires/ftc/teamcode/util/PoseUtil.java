package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public class PoseUtil {
    public static Pose2d toRRPose2d(com.arcrobotics.ftclib.geometry.Pose2d pose2d){
        return new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }
    public static com.arcrobotics.ftclib.geometry.Pose2d toFTCLibPos2d(Pose2d pose2d){
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getHeading()));
    }
    public static Pose2d toRRPoseVelo(ChassisSpeeds velo){
        return new Pose2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond, velo.omegaRadiansPerSecond);
    }
    public static Pose2d toInches(Pose2d pose) {
        return new Pose2d(pose.getX()* 39.3701, pose.getY() * 39.701, pose.getHeading());
    }
    public static Pose2d toMeters(Pose2d pose) {
        return new Pose2d(pose.getX() / 39.3701, pose.getY() / 39.701, pose.getHeading());
    }
    public static com.arcrobotics.ftclib.geometry.Pose2d toMeters(com.arcrobotics.ftclib.geometry.Pose2d pose) {
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose.getX() / 39.3701, pose.getY() / 39.701, new Rotation2d(pose.getHeading()));
    }
}
