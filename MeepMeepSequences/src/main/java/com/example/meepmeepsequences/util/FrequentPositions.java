package com.example.meepmeepsequences.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static com.example.meepmeepsequences.util.Details.alliance;
import static com.example.meepmeepsequences.util.Details.location;
import static com.example.meepmeepsequences.util.Details.side;

public class FrequentPositions {
    public static Pose2d flipSide(Pose2d pose2d) {
        return new Pose2d(pose2d.getX(), -pose2d.getY(), -pose2d.getHeading());
    }

    public static Pose2d startingPosition() {
        Pose2d regular = side == Side.CYCLING ? new Pose2d(8.2, -58, Math.toRadians(-90)) : new Pose2d(-36, -58, Math.toRadians(-90));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d dumpPosition() {
        Pose2d regular = side == Side.CYCLING ? new Pose2d(6.0, -32.0, Math.toRadians(-30.0)) : new Pose2d(-28.0, -31.0, Math.toRadians(20.0));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d cycleDumpPosition() {
        Pose2d regular = side == Side.CYCLING ? new Pose2d(6.0, -32.0, Math.toRadians(-30.0)) : new Pose2d();
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d duckLocation() {
        Pose2d[] arr = duckLocations();
        switch (location) {
            case LEFT: return arr[0];
            case RIGHT: return arr[2];
            case MIDDLE: return arr[1];
        }
        return arr[0];
    }

    public static Pose2d[] duckLocations() {
        if (alliance == Alliance.RED) {
            if (side == Side.CAROUSEL) {
                return new Pose2d[]{
                        new Pose2d(-32.0, -50.0, Math.toRadians(0)),
                        new Pose2d(-40.0, -50.0, Math.toRadians(0)),
                        new Pose2d(-50.0, -50.0, Math.toRadians(0))
                };
            } else {
                return new Pose2d[]{
                        new Pose2d(8.2, -50.0, Math.toRadians(40)),
                        new Pose2d(8.2, -50.0, Math.toRadians(10)),
                        new Pose2d(9, -47.0, Math.toRadians(-20))
                };
            }
        } else {
            if (side == Side.CAROUSEL) {
                return new Pose2d[]{
                        flipSide(new Pose2d(-32.0, 50.0, Math.toRadians(0))),
                        flipSide(new Pose2d(-40.0, 50.0, Math.toRadians(0))),
                        flipSide(new Pose2d(-50.0, 50.0, Math.toRadians(0)))
                };
            } else {
                return new Pose2d[]{
                        new Pose2d(8.2, 50.0, Math.toRadians(70)),
                        new Pose2d(8.2, 50.0, Math.toRadians(30)),
                        new Pose2d(9, 47.0, Math.toRadians(70))
                };
            }
        }
    }
}
