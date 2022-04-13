package org.firstinspires.ftc.teamcode.util.controllers

class MovingMedian @JvmOverloads constructor (private val arraySize: Int = 5) {
    private val movingStats: ArrayList<Double> = ArrayList()
    fun update(i: Double): Double {
        if (i.isNaN()) return movingStats.sorted()[movingStats.size / 2]
        movingStats.add(i)
        if (movingStats.size >= arraySize) movingStats.removeAt(0)
        return movingStats.sorted()[movingStats.size / 2]
    }
    init {
        movingStats.add(0.0)
    }
}