package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import java.util.ArrayList;

public class Line {
    ArrayList<Coordinate> list = new ArrayList<Coordinate>();
    double slope;
    double constant;
    public Line(Coordinate start, Coordinate end){
        list.add(start);
        list.add(end);
    }
    public double getDesiredY(double x){
      return slope*x + constant;
    }
    public double getDesiredX(double y){
      return (y- constant)/slope;
    }

    public Coordinate getClosestPt(Coordinate curr){
        return list.get(1);
    }
    public void addPoint(Coordinate n){
        list.add(n);
    }
}
