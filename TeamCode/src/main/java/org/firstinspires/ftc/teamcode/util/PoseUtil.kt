package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
    fun Pose2d.toRRPose2d(): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(this.x, this.y, this.heading)
    }

    fun com.acmerobotics.roadrunner.geometry.Pose2d.toFTCLibPose2d(): Pose2d {
        return Pose2d(this.x, this.y, Rotation2d(this.heading))
    }

    fun ChassisSpeeds.toRRPoseVelo(): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            this.vxMetersPerSecond,
            this.vyMetersPerSecond,
            this.omegaRadiansPerSecond
        )
    }

    fun com.acmerobotics.roadrunner.geometry.Pose2d.toInches(): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            this.x * 39.3701,
            this.y * 39.701,
            this.heading
        )
    }

    fun com.acmerobotics.roadrunner.geometry.Pose2d.toMeters(): com.acmerobotics.roadrunner.geometry.Pose2d {
        return com.acmerobotics.roadrunner.geometry.Pose2d(
            this.x / 39.3701,
            this.y / 39.701,
            this.heading
        )
    }

    fun Pose2d.toMeters(): Pose2d {
        return Pose2d(this.x / 39.3701, this.y / 39.701, Rotation2d(this.heading))
    }