package org.firstinspires.ftc.teamcode.modules.wrappers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class ControllableMotor(vararg motors: DcMotorEx) : Actuator {
    private var motors: Array<DcMotorEx> = motors as Array<DcMotorEx>
    private var disabled = false

    var power: Double
        get() = motors[0].power
        set(var1) {
            if (disabled) motors.forEach { it.power = 0.0 }
            motors.forEach { it.power = var1 }
        }
    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() = motors[0].zeroPowerBehavior
        set(var1) {
            motors.forEach { it.zeroPowerBehavior = var1 }
        }
    var direction: DcMotorSimple.Direction
        get() = motors[0].direction
        set(var1) {
            motors.forEach { it.direction = var1 }
        }
    var mode: DcMotor.RunMode
        get() = motors[0].mode
        set(var1) {
            motors.forEach { it.mode = var1 }
        }
    fun getCurrent(unit: CurrentUnit): Double {
        return motors
            .map { it.getCurrent(unit) }
            .reduce { a, b -> a + b }
    }


    override fun disable() {
        disabled = true
    }

    override fun enable() {
        disabled = false
    }
}