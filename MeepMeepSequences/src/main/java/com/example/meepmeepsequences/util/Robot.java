package com.example.meepmeepsequences.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Robot {
    public static int windowSize = 1000;
    public static Alliance alliance = Alliance.BLUE;
    public static Side side = Side.CAROUSEL;

    public static Pose2d flipSide(Pose2d pose2d) {
        return new Pose2d(pose2d.getX(), -pose2d.getY(), -pose2d.getHeading());
    }

    public static Pose2d startingPosition() {
        Pose2d regular = side == Side.CYCLING ? new Pose2d(12, -58, Math.toRadians(-90)) : new Pose2d(-36, -58, Math.toRadians(-90));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d dumpPosition() {
        Pose2d regular = side == Side.CYCLING ? new Pose2d(1.0, -40.0, Math.toRadians(-60.0)) : new Pose2d(-28.0, -31.0, Math.toRadians(20.0));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d cycleDumpPosition() {
        Pose2d regular = side == Side.CYCLING ? new Pose2d(1.0, -40.0, Math.toRadians(-60.0)) : new Pose2d();
        return alliance == Alliance.RED ? regular : flipSide(regular);
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
                        new Pose2d(3.0, -50.0),
                        new Pose2d(11.5, -50.0),
                        new Pose2d(20.0, -50.0)
                };
            }
        } else {
            if (side == Side.CAROUSEL) {
                return new Pose2d[]{
                        flipSide(new Pose2d(-32.0, -50.0, Math.toRadians(0))),
                        flipSide(new Pose2d(-40.0, -50.0, Math.toRadians(0))),
                        flipSide(new Pose2d(-50.0, -50.0, Math.toRadians(0)))
                };
            } else {
                return new Pose2d[]{
                        new Pose2d(3.0, -50.0),
                        new Pose2d(11.5, -50.0),
                        new Pose2d(20.0, -50.0)
                };
            }
        }
    }
}
