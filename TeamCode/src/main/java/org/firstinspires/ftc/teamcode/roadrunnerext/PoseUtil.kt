package org.firstinspires.ftc.teamcode.roadrunnerext

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds

fun Pose2d.toRRPose2d(): com.acmerobotics.roadrunner.geometry.Pose2d {
    return com.acmerobotics.roadrunner.geometry.Pose2d(this.x, this.y, this.heading)
}

fun com.acmerobotics.roadrunner.geometry.Pose2d.toFTCLibPose2d(): Pose2d {
    return Pose2d(this.x, this.y, Rotation2d(this.heading))
}

fun com.acmerobotics.roadrunner.geometry.Pose2d.flipHeading(): com.acmerobotics.roadrunner.geometry.Pose2d {
    return com.acmerobotics.roadrunner.geometry.Pose2d(this.x, this.y, this.heading + Math.PI)
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

fun Double.flip(negative: Boolean): Double {
    return if (negative) -this else this
}

fun com.acmerobotics.roadrunner.geometry.Pose2d.flip(negative: Boolean): com.acmerobotics.roadrunner.geometry.Pose2d {
    return if (negative) com.acmerobotics.roadrunner.geometry.Pose2d(this.x, -this.y, -this.heading) else this
}

fun Vector2d.flip(negative: Boolean): Vector2d {
    return if (negative) Vector2d(this.x, -this.y) else this
}

fun Vector2d.polarAdd (distance: Double, angle: Double) : Vector2d {
    return Coordinate.toPoint(this).polarAdd(distance, angle).toVector()
}

fun com.acmerobotics.roadrunner.geometry.Pose2d.polarAdd (distance: Double) : com.acmerobotics.roadrunner.geometry.Pose2d {
    return Coordinate.toPoint(this).polarAdd(distance, this.heading).toPose2d(this.heading)
}
fun Vector2d.toPose (angle: Double) : com.acmerobotics.roadrunner.geometry.Pose2d {
    return com.acmerobotics.roadrunner.geometry.Pose2d(this, angle)
}
