package org.firstinspires.ftc.teamcode.modules.wrappers.actuators

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.util.Encoder
import kotlin.math.abs
import kotlin.math.round

class ControllableServos(vararg servos: Servo) :
    Actuator {
    private var timer = ElapsedTime()
    private var servos: Array<Servo> = servos as Array<Servo>
    private var previousPosition = 0.0

    var servoRotation = Math.toRadians(270.0)
    var gearing = 1.0

    var positionPerSecond = 0.7
    private var incrementingPosition = true
    private var initted = false
    var encoder: Encoder? = null
    var isMotionProfiled = false

    var lowerLimit = 0.0
        private set
    var upperLimit = 1.0
        private set

    private var angleOffset = 0.0
    private var posOffset = 0.0

    fun setLimits(lowerLimit: Double, upperLimit: Double) {
        this.lowerLimit = lowerLimit
        this.upperLimit = upperLimit
    }
    private fun ticksToDegrees(ticks: Int): Double {
        return ticks / 8192.0
    }
    val realPosition: Double?
        get() {
            if (encoder == null) return null
            return getPosition(ticksToDegrees(encoder!!.currentPosition) / gearing) - posOffset
        }
    val estimatedPosition: Double
        get() = round((if (incrementingPosition) Range.clip(
            previousPosition + timer.seconds() * positionPerSecond,
            previousPosition,
            position
        ) else Range.clip(
            previousPosition - timer.seconds() * positionPerSecond,
            position,
            previousPosition
        )) * 1000) / 1000
    var position: Double
        get() = servos[0].position
        set(var1) {
            val position = position
            if (round(position * 1000) / 1000 == round(var1 * 1000) / 1000 && initted) {
                servos.forEach { it.position = Range.clip(round(var1 * 1000) / 1000, lowerLimit, upperLimit) }
                return
            }
            initted = true
            incrementingPosition = estimatedPosition < var1
            previousPosition = estimatedPosition
            servos.forEach { it.position = Range.clip(round(var1 * 1000) / 1000, lowerLimit, upperLimit) }
            timer.reset()
        }

    val error: Double
        get() = abs(estimatedPosition - position)
    var reversedAngle = false
    var angle: Double
        set(value) {
            position = getPosition(value)
        }
        get() = getAngle(if (reversedAngle) 1 - position else position)
    val estimatedAngle: Double
        get() = getAngle(estimatedPosition)
    val realAngle: Double?
        get() = realPosition?.let(::getAngle)


    private fun getAngle(position: Double) = (position * servoRotation * gearing) - angleOffset
    private fun getPosition(angle: Double) = (angle + angleOffset) / (servoRotation * gearing)

    fun lock() {
        val realPosition = estimatedPosition
        for (servo in servos) {
            servo.position = realPosition
        }
    }
    fun calibrateOffset(position: Double, angle: Double) {
        // angle = (position * servoRotation) - angleOffset
        // angleOffset = angle - pos*servoRot
        // position = (angle + angleOffset) / servoRotation
        angleOffset = angle - position * servoRotation
        posOffset = position
        encoder?.reset()
    }

    fun init(var1: Double) {
        position = var1
        previousPosition = var1
        posOffset = var1
    }

    val isTransitioning: Boolean
        get() = round(estimatedPosition * 1000) / 1000 != round(position * 1000) / 1000

    override fun disable() {
//        servos.forEach { if ((it as? ServoImplEx)?.isPwmEnabled == true) it.setPwmDisable() }
    }

    override fun enable() {
//        servos.forEach { if ((it as? ServoImplEx)?.isPwmEnabled == false) it.setPwmEnable() }
    }

    fun increaseRange(newRange: PwmControl.PwmRange) {
        servos.forEach { (it as ServoImplEx).pwmRange = newRange }
    }

    override fun update() {
    }
}