package org.firstinspires.ftc.teamcode.roadrunnerext.geometry

import com.acmerobotics.roadrunner.geometry.Vector2d

class Circle(var center: Vector2d, var radius: Double) {
    fun expandedRadius(radiusOffset: Double) = Circle(center, radius + radiusOffset)
    fun getTangentPointToPoint(point: Vector2d) {
        // y = mx + b
        // (x - center.x)^2 + (y - center.y)^2 = r^2
        // (x - center.x)^2 + ((mx + b)- center.y)^2 = r^2
        // x^2 + center.x^2 - 2 * x * center.x
    }
}