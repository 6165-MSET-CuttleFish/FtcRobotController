package org.firstinspires.ftc.teamcode.roadrunnerext.geometry

import com.acmerobotics.roadrunner.geometry.Vector2d
import kotlin.math.sqrt

class Vector3d @JvmOverloads constructor(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0,
) {
    fun norm(): Double = sqrt(x * x + y * y + z * z)
    infix fun distTo(other: Vector3d): Double = (this - other).norm()
    infix fun dot(other: Vector3d) = x * other.x + y * other.y + z * other.z
    fun horizontalAngleTo(other: Vector3d) = component1().angleBetween(other.component1())
    fun verticalAngleTo(other: Vector3d) = component2().angleBetween(other.component2())
    fun component1() = Vector2d(x, y)
    fun component2() = Vector2d(x, z)
    fun component3() = Vector2d(y, z)
    operator fun plus(other: Vector3d) =
        Vector3d(x + other.x, y + other.y, z + other.z)

    operator fun minus(other: Vector3d) =
        Vector3d(x - other.x, y - other.y, z - other.z)

    operator fun times(scalar: Double) = Vector3d(scalar * x, scalar * y, scalar * z)

    operator fun div(scalar: Double) = Vector3d(x / scalar, y / scalar, scalar * z)

    operator fun unaryMinus() = Vector3d(-x, -y, -z)

    operator fun Double.times(vector: Vector3d) = vector * this

    operator fun Double.div(vector: Vector3d) = vector / this
}