package org.firstinspires.ftc.teamcode.util.controllers

class LowPassFilter(var a: Double, var p: Double) {
    fun update(i: Double): Double {
        val estimate = a * p + (1 - a) * i
        p = estimate
        return estimate
    }
}