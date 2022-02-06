package org.firstinspires.ftc.teamcode.modules.wrappers

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import kotlin.math.round

class ControllableServos(vararg servos: Servo) : Actuator {
    private var timer = ElapsedTime()
    private var servos: Array<Servo> = servos as Array<Servo>
    private var previousPosition = 0.0
    var positionPerSecond = 0.7
    private var incrementingPosition = true
    private var initted = false
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
            if (round(position * 1000) / 1000 == round(var1 * 1000) / 1000 && initted) return
            initted = true
            incrementingPosition = realPosition < var1
            previousPosition = realPosition
            servos.forEach { it.position = round(var1 * 1000) / 1000 }
            timer.reset()
        }

    fun init(var1: Double) {
        position = var1
        previousPosition = var1
    }

    val isTransitioning: Boolean
        get() = round(realPosition * 1000) / 1000 != round(position * 1000) / 1000

    override fun disable() {
        servos.forEach { if ((it as? ServoImplEx)?.isPwmEnabled == true) it.setPwmDisable() }
    }

    override fun enable() {
        servos.forEach { if ((it as? ServoImplEx)?.isPwmEnabled == false) it.setPwmEnable() }
    }
}