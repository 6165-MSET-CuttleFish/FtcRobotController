package org.firstinspires.ftc.teamcode.roadrunnerext.geometry;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Line {
    Vector2d start;
    Vector2d end;

    public Line(Vector2d start, Vector2d end) {
        this.start = start;
        this.end = end;
    }

    public static Line yAxis(double y) {
        return new Line(new Vector2d(-74.4, y), new Vector2d(74.4, y));
    }

    public static Line xAxis(double x) {
        return new Line(new Vector2d(x, -74.4), new Vector2d(x, -74.4));
    }

    public Vector2d getStart() {
        return start;
    }

    public Vector2d getEnd() {
        return end;
    }
}
