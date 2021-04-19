package org.firstinspires.ftc.teamcode.PurePursuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Coordinate {
    public double x;
    public double y;

    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate(Coordinate point) {
        this.x = point.getX();
        this.y = point.getY();
    }
    public Coordinate(Pose2d pose){
        this.x = pose.getX();
        this.y = pose.getY();
    }
    public Coordinate(){}
    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Coordinate toPoint(){
        return new Coordinate(x, y);
    }
    public void setPoint(Coordinate point) {
        this.x = point.getX();
        this.y = point.getY();
    }
    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double[] getPoint() {
        double[] arr = {x, y};
        return arr;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
    @Override
    public String toString() {
        return "[" + x + ", " + y + "]";
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
    public void polarAdd(double angle, double distance){
        add(xCovered(angle, distance), yCovered(angle, distance));
    }
    public double angleTo(Coordinate desired) {
        double x = desired.getX() - getX();
        double y = desired.getY() - getY();
        double angle = Math.toDegrees(Math.atan2(y, x));
        if(angle > 180){
            angle -= 360;
        }
        else if(angle < -180){
            angle += 360;
        }
        return angle;
    }
    public double curveDistanceTo(Coordinate C, Coordinate B){
        double a = C.distanceTo(B);
        double c = distanceTo(B);
        return a+c;
        //return Math.abs(C.getX() - getX()) + Math.abs(C.getY() - getY());
    }
    public double getMatchX(Coordinate C, double heading){
        if(heading > 180){
            heading -= 360;
        }
        else if(heading < -180){
            heading += 360;
        }
        double x = C.getX() - getX();
        return x/Math.cos(Math.toRadians(heading));
    }
    public double getMatchY(Coordinate C, double heading){//verify
        if(heading > 180){
            heading -= 360;
        }
        else if(heading < -180){
            heading += 360;
        }
        double y = C.getY() - getY();
        return y/Math.sin(Math.toRadians(heading));
    }
    public static double xCovered(double angle, double distance) {
        return Math.cos(angle) * distance;
    }

    public static double yCovered(double angle, double distance) {
        return Math.sin(angle) * distance;
    }
    public static Coordinate toPoint(Coordinate pt){
        return new Coordinate(pt.x, pt.y);
    }
    public static Coordinate toPoint(Pose2d pt){
        return new Coordinate(pt.getX(), pt.getY());
    }
    public static Coordinate toPoint(Vector2d pt){
        return new Coordinate(pt.getX(), pt.getY());
    }
    public static Vector2d pointLineIntersection(Pose2d pose2d, double x){
        //y = mx + b
        double m = Math.tan(pose2d.getHeading());
        double b = pose2d.getY() - m*pose2d.getX();
        double intersectionY = m*x + b;
        return new Vector2d(x, intersectionY);
    }
    public static double distanceToLine(Pose2d pose2d, double x){
        return pose2d.vec().distTo(pointLineIntersection(pose2d, x));
    }
}