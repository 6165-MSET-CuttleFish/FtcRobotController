package org.firstinspires.ftc.teamcode.roadrunnerext.geometry

import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.hypot

class Vector3d @JvmOverloads constructor(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0,
) {
    fun distTo(other: Vector3d): Double {
        val a = Vector2d(x, y).distTo(Vector2d(other.x, other.y))
        val b = Vector2d(x, z).distTo(Vector2d(other.x, other.z))
        return hypot(a, b)
    }
}