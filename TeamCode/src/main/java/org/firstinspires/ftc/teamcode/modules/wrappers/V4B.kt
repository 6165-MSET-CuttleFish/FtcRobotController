package org.firstinspires.ftc.teamcode.modules.wrappers

import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import kotlin.math.acos
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.sin

class V4B(private val armLength: Double, val servos: ControllableServos) {
    val vector: Vector2d
        get() = Vector2d(armLength * cos(servos.angle), armLength * sin(servos.angle))
    var position: Double
        set(value) {
            servos.position = value
        }
        get() = servos.position
    val realPosition: Double
        get() = servos.estimatedPosition
    val isTransitioning: Boolean
        get() = servos.isTransitioning
    fun setY(height: Double) {
        servos.angle = asin(height / armLength)
    }
    fun setX(length: Double) {
        servos.angle = acos(length / armLength)
    }
}