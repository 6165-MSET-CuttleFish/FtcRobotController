package org.firstinspires.ftc.teamcode.modules.wrappers.mechanisms

import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class Linkage @JvmOverloads constructor(
    private val a: Double,
    private val b: Double,
    private val heightConstraint: Double,
    val servos: ControllableServos,
    private val initialAngle: Double = 0.0,
) {
    val estimatedDisplacement: Double
        get() = -displacement(servos.estimatedAngle) + displacement(initialAngle)
    val displacement: Double
        get() = -displacement(servos.angle) + displacement(initialAngle)
    private fun displacement(theta: Double): Double =
        a * cos(theta) - sqrt(b.pow(2) - (a * sin(theta) * heightConstraint).pow(2))
    var position: Double
        get() = servos.position
        set(value) {
            servos.position = value
        }
    val estimatedPosition: Double
        get() = servos.estimatedPosition
    val isTransitioning: Boolean
        get() = servos.isTransitioning
}