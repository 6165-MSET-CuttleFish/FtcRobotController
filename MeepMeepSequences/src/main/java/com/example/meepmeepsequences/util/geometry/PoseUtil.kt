package com.example.meepmeepsequences.util.geometry

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d


fun Pose2d.toInches(): Pose2d {
    return Pose2d(
        this.x * 39.3701,
        this.y * 39.701,
        this.heading
    )
}

fun Double.flip(negative: Boolean): Double {
    return if (negative) -this else this
}

fun Pose2d.flip(negative: Boolean): Pose2d {
    return if (negative) Pose2d(this.x, -this.y, -this.heading) else this
}

fun Vector2d.flip(negative: Boolean): Vector2d {
    return if (negative) Vector2d(this.x, -this.y) else this
}

fun Line.flip(negative: Boolean) : Line {
    return Line(this.start.flip(negative), this.end.flip(negative))
}

fun Vector2d.polarAdd (distance: Double, angle: Double) : Vector2d {
    return Coordinate.toPoint(this).polarAdd(distance, angle).toVector()
}

fun Pose2d.polarAdd (distance: Double, angleOffset: Double = 0.0) : Pose2d {
    return Coordinate.toPoint(this).polarAdd(distance, this.heading + angleOffset).toPose2d(this.heading)
}

fun Vector2d.toPose (angle: Double) : Pose2d {
    return Pose2d(this, angle)
}
fun Vector2d.angleTo (vector2d: Vector2d): Double {
    return Coordinate.toPoint(this).angleTo(
        Coordinate.toPoint(vector2d))
}
