package com.example.meepmeepsequences.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Robot {
    public static Alliance alliance = Alliance.BLUE;
    public static Side side = Side.CAROUSEL;

    public static Pose2d flipSide(Pose2d pose2d) {
        return new Pose2d(pose2d.getX(), -pose2d.getY(), -pose2d.getHeading());
    }

    public static Pose2d midPoint() {
        Pose2d regular = new Pose2d(-55.0, -55.0, Math.toRadians(-210.0));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d startingPosition() {
        Pose2d regular = new Pose2d(-36, -58, Math.toRadians(-90));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d cycleDump() {
        Pose2d regular = new Pose2d(-28.0, -31.0, Math.toRadians(20.0));
        return alliance == Alliance.RED ? regular : flipSide(regular);
    }

    public static Pose2d[] duckLocations() {
        Pose2d[] values;
        if (side == Side.CAROUSEL) {
            values = new Pose2d[] {
                    new Pose2d(-32.0, -50.0, Math.toRadians(0)),
                    new Pose2d(-40.0, -50.0, Math.toRadians(0)),
                    new Pose2d(-50.0, -50.0, Math.toRadians(0))
            };
        } else {
            values = new Pose2d[] {
                    new Pose2d(3.0, -50.0),
                    new Pose2d(11.5, -50.0),
                    new Pose2d(20.0, -50.0)
            };
        }
        if (alliance == Alliance.BLUE) {
            for (int i = 0; i < values.length; i++) {
                values[i] = flipSide(values[i]);
            }
        }
        return values;
    }
}
