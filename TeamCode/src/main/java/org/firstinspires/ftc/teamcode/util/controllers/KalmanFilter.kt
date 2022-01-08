package org.firstinspires.ftc.teamcode.util.controllers

class KalmanFilter @JvmOverloads constructor(
    var x: Double = 0.0, // initial state
    var Q: Double = 0.1, // your model covariance
    var R: Double = 0.4, // your sensor covariance
    var p: Double = 1.0, // your initial covariance guess
    var K: Double = 1.0, // your initial kalman gain guess
    var x_previous: Double = x,
    var p_previous: Double = p,
){
    fun setState(state: Double) {
        this.x = state
        this.x_previous = x
    }
    fun update(u: Double, z: Double): Double {
        x = x_previous + u
        p = p_previous + Q
        K = p / (p + R)
        // use another sensor for z
        x += K * (z - x)
        p = 1 - K * p
        x_previous = x
        p_previous = p
        return x
    }
}