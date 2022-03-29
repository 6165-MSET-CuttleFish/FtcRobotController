package org.firstinspires.ftc.teamcode.modules.wrappers

import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class Linkage(private val a: Double,
              private val b: Double,
              private val heightConstraint: Double,
              val servos: ControllableServos
              ) {
    var displacement: Double
        set(value) {
            servos.position = value
        }
        get() = sqrt(b.pow(2.0) - (heightConstraint - a*sin(servos.angle)))
    var position: Double
        get() = servos.position
        set(value) {
            servos.position = value
        }
    val realPosition: Double
        get() = servos.realPosition
    val isTransitioning: Boolean
        get() = servos.isTransitioning
}