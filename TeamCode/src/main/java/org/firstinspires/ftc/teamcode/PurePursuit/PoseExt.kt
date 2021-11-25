package org.firstinspires.ftc.teamcode.PurePursuit

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

fun Vector2d.polarAdd (distance: Double, angle: Double) : Vector2d {
    return Coordinate.toPoint(this).polarAdd(distance, angle).toVector()
}

fun Pose2d.polarAdd (distance: Double) : Vector2d {
    return Coordinate.toPoint(this).polarAdd(distance, this.heading).toVector()
}