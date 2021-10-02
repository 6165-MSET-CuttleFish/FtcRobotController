package org.firstinspires.ftc.teamcode.PurePursuit

import com.acmerobotics.roadrunner.geometry.Vector2d

fun Vector2d.polarAdd (distance: Double, angle: Double) : Vector2d {
    return Coordinate.toPoint(this).polarAdd(distance, angle).toVector()
}