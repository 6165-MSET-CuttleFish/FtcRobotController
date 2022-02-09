package org.firstinspires.ftc.teamcode.util.controllers

import com.qualcomm.robotcore.util.Range

class LowPassFilter(a: Double, var p: Double) {
    fun update(i: Double): Double {
        if (i.isNaN()) return p
        val estimate = a * p + (1 - a) * i
        p = estimate
        return estimate
    }
    var a: Double = 0.0
        set (value) {
            field = Range.clip(value, 0.0, 1.0)
        }
    init {
        this.a = a
    }
}