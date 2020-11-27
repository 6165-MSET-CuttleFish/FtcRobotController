package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

public class Goal extends Coordinate {
    public double height;
    public Goal(Coordinate c, double height) {
        super(c);
        this.height = height;
    }
    public Goal(double x, double y, double z){
        super(x,y);
        height = z;
    }
}
