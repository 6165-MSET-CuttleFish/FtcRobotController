package org.firstinspires.ftc.teamcode.roadrunnerext.geometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class Coordinate {
    public double x;
    public double y;
    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate toPoint(){
        return new Coordinate(x, y);
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }

    public void addX(double x) {
        this.x += x;
    }
    public void addY(double y) {
        this.y += y;
    }
    public void add(double x, double y) {
        addX(x);
        addY(y);
    }
    public Pose2d toPose2d(double angle){
        return new Pose2d(x, y, angle);
    }
    public static Pose2d toPose(Vector2d vector2d, double angle){
        return new Pose2d(vector2d.getX(), vector2d.getY(), angle);
    }
    public double distanceTo(Coordinate B) {
        return Math.sqrt(Math.pow(B.getX() - getX(), 2) + Math.pow(B.getY() - getY(), 2));
    }
    public Coordinate polarAdd(double distance, double angle){
        add(xCovered(angle, distance), yCovered(angle, distance));
        return this;
    }
    public double angleTo(Coordinate desired) {
        double x = desired.getX() - getX();
        double y = desired.getY() - getY();
        double angle = Math.atan2(y, x);
        if(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        else if(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        return angle;
    }
    public static double xCovered(double angle, double distance) {
        return Math.cos(angle) * distance;
    }

    public static double yCovered(double angle, double distance) {
        return Math.sin(angle) * distance;
    }

    public Vector2d toVector(){
        return new Vector2d(x, y);
    }
    public static Coordinate toPoint(Pose2d pt){
        return new Coordinate(pt.getX(), pt.getY());
    }
    public static Coordinate toPoint(Vector2d pt){
        return new Coordinate(pt.getX(), pt.getY());
    }
    public static Vector2d pointLineIntersection(Pose2d pose2d, double x){
        double m = Math.tan(pose2d.getHeading());
        double b = pose2d.getY() - m*pose2d.getX();
        double intersectionY = m*x + b;
        return new Vector2d(x, intersectionY);
    }
    public static double distanceToLine(Pose2d pose2d, double x){
        return pose2d.vec().distTo(pointLineIntersection(pose2d, x));
    }

    public static ArrayList<Vector2d> lineCircleIntersection(Circle circle,
                                                             Coordinate lp1, Coordinate lp2){
        if(Math.abs(lp1.getY() - lp2.getY()) < 0.003){
            lp1.setY(lp2.getY() + 0.003);
        }
        if(Math.abs(lp1.getX() - lp2.getX()) < 0.003){
            lp1.setX(lp2.getX() + 0.003);
        }
        double m1 = (lp2.getY() - lp1.getY())/(lp2.getX() - lp1.getX());

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = lp1.getX() - circle.getCenter().getX();
        double y1 = lp1.getY() - circle.getCenter().getY();

        double quadraticB = (2.0 * m1 * y1) - (2.0 - pow(m1, 2) * x1);

        double quadraticC = ((pow(m1,2) * pow(x1,2))) - (2.0*y1*m1*x1) + pow(y1, 2) - pow(circle.getRadius(), 2);

        ArrayList<Vector2d> allCoordinates = new ArrayList<>();
        try {
            final double determinant = pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC);
            double xRoot1 = (-quadraticB + sqrt(determinant)) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circle.getCenter().getX();
            yRoot1 += circle.getCenter().getY();
            double minX = Math.min(lp1.getX(), lp2.getX());
            double maxX = Math.max(lp1.getX(), lp2.getX());

            if (xRoot1 > minX && xRoot1 < maxX) {
                allCoordinates.add(new Vector2d(xRoot1, yRoot1));
            }
            double xRoot2 = (-quadraticB - sqrt(determinant)) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circle.getCenter().getX();
            yRoot2 += circle.getCenter().getY();
            if (xRoot2 > minX && xRoot2 < maxX) {
                allCoordinates.add(new Vector2d(xRoot2, yRoot2));
            }
        } catch (Exception ignored) {

        }
        return allCoordinates;
    }
}