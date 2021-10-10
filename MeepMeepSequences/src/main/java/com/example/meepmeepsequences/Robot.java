package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Robot {
    public static boolean blue = false;
    public static Vector2d[] duckLocations(){
        if (blue) {
            return new Vector2d[]{
                    new Vector2d(3.0, -50.0),
                    new Vector2d(3.0, -35.0),
                    new Vector2d(3.0, -35.0)
            };
        }
        return new Vector2d[]{
                new Vector2d(3.0, -50.0),
                new Vector2d(11.5, -50.0),
                new Vector2d(20.0, -50.0)
        };
    }
}
