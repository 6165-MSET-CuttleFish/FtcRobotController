package com.example.meepmeepsequences.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Robot {
    public static boolean blue = false;
    public static Alliance alliance = Alliance.BLUE;
    public static Side side = Side.CAROUSEL;
    public static boolean carousel = true;
    public static Pose2d[] duckLocations(){
        if (alliance == Alliance.BLUE) {
            if (side == Side.CAROUSEL) {
                return new Pose2d[]{
                        new Pose2d(3.0, -50.0),
                        new Pose2d(3.0, -35.0),
                        new Pose2d(3.0, -35.0)
                };
            }
            return new Pose2d[]{
                    new Pose2d(3.0, -50.0),
                    new Pose2d(3.0, -35.0),
                    new Pose2d(3.0, -35.0)
            };
        }
        if (side == Side.CAROUSEL) {
            return new Pose2d[]{
                    new Pose2d(-32.0, -50.0, Math.toRadians(0)),
                    new Pose2d(-40.0, -50.0, Math.toRadians(0)),
                    new Pose2d(-50.0, -50.0, Math.toRadians(0))
            };
        }
        return new Pose2d[]{
                new Pose2d(3.0, -50.0),
                new Pose2d(11.5, -50.0),
                new Pose2d(20.0, -50.0)
        };
    }
}
