package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class PoseUtil {
    public static Pose2d toRRPos2d(com.arcrobotics.ftclib.geometry.Pose2d pose2d){
        return new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }
    public static com.arcrobotics.ftclib.geometry.Pose2d toFTCLibPos2d(Pose2d pose2d){
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getHeading()));
    }
}
