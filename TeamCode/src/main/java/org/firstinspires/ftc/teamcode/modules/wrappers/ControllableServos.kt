package org.firstinspires.ftc.teamcode.modules.wrappers

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class ControllableServos(vararg servos: Servo) {
    private var timer = ElapsedTime()
    private var servos: Array<Servo> = servos as Array<Servo>
    var previousPosition = 0.0
    var totalMotionDuration = 1.0
    var incrementingPosition = true
    private val realPosition: Double
        get() = if (incrementingPosition) Range.clip(
            previousPosition + timer.seconds() / totalMotionDuration,
            previousPosition,
            servos[0].position
        ) else Range.clip(
            previousPosition - timer.seconds() / totalMotionDuration,
            servos[0].position,
            previousPosition
        )

    fun lock() {
        val realPosition = realPosition
        for (servo in servos) {
            servo.position = realPosition
        }
    }

    var position: Double
        get() = servos[0].position
        set(var1) {
            for (servo in servos) {
                servo.position = var1
            }
            if (servos[0].position == var1) return
            incrementingPosition = realPosition < var1
            previousPosition = realPosition
            timer.reset()
        }
    val isTransitioning: Boolean
        get() = realPosition != servos[0].position

}