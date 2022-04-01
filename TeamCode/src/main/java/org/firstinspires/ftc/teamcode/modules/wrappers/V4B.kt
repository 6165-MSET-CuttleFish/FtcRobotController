package org.firstinspires.ftc.teamcode.modules.wrappers

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class V4B(private val armLength: Double, private val servos: ControllableServos) {
    var displacement: Pose2d
        set(value) {
            servos.position = value.x
        }
        get() = Pose2d(armLength * cos(servos.angle), armLength * sin(servos.angle))
    val position: Double
        get() = servos.position
    val realPosition: Double
        get() = servos.realPosition
    val isTransitioning: Boolean
        get() = servos.isTransitioning
}