package org.firstinspires.ftc.teamcode.modules.wrappers

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import kotlin.math.round

class ControllableServos(vararg servos: Servo) {
    var timer = ElapsedTime()
    private var servos: Array<Servo> = servos as Array<Servo>
    var previousPosition = 0.0
    var positionPerSecond = 1.0
    var incrementingPosition = true
    val realPosition: Double
        get() = round((if (incrementingPosition) Range.clip(
            previousPosition + timer.seconds() * positionPerSecond,
            previousPosition,
            position
        ) else Range.clip(
            previousPosition - timer.seconds() * positionPerSecond,
            position,
            previousPosition
        )) * 1000) / 1000

    fun lock() {
        val realPosition = realPosition
        for (servo in servos) {
            servo.position = realPosition
        }
    }

    var position: Double
        get() = servos[0].position
        set(var1) {
            if (round(position * 1000) / 1000 == round(var1 * 1000) / 1000) return
            previousPosition = realPosition
            servos.forEach { it.position = var1 }
            incrementingPosition = realPosition < var1
            timer.reset()
        }
    val isTransitioning: Boolean
        get() = round(realPosition * 1000) / 1000 != round(position * 1000) / 1000

}