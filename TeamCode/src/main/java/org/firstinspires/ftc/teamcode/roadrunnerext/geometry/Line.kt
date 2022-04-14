package org.firstinspires.ftc.teamcode.roadrunnerext.geometry

import com.acmerobotics.roadrunner.geometry.Vector2d

class Line(var start: Vector2d, var end: Vector2d) {
    companion object {
        fun yAxis(y: Double): Line {
            return Line(Vector2d(-74.4, y), Vector2d(74.4, y))
        }

        fun xAxis(x: Double): Line {
            return Line(Vector2d(x, -74.4), Vector2d(x, -74.4))
        }
    }
    val slope: Double
        get() {
            val delta = end - start
            return delta.y / delta.x
        }
    val yIntercept: Double
        get() {
            val slope = slope
            // b = y - mx
            return end.y - slope * end.x
        }
}