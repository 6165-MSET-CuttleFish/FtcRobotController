package org.firstinspires.ftc.teamcode.modules.wrappers

import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import kotlin.math.hypot

class V4B(private val lever: Double, private val moment: Double, private val servos: ControllableServos) {
    var displacement: Double
        set(value) {
            servos.position = value
        }
        get() = hypot(lever, moment)
    val position: Double
        get() = servos.position
    val realPosition: Double
        get() = servos.realPosition
    val isTransitioning: Boolean
        get() = servos.isTransitioning
}