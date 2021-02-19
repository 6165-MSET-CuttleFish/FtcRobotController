package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate

fun Pose2d.toPoint() : Coordinate {
    return Coordinate.toPoint(this)
}