package org.firstinspires.ftc.teamcode.roadrunnerext.geometry;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Circle {
    Vector2d center;
    double radius;

    public Circle(Vector2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public Vector2d getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }
}