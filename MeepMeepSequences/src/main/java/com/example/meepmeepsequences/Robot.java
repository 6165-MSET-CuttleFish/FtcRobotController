package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Robot {
    static boolean blue = true;
    public static Vector2d goal() {
        if (blue) {
            return new Vector2d(70.5275, 37.9725);
        }
        return new Vector2d(70.5275, -37.9725);
    }
    public static Vector2d pwrShotLocal() {
        if (blue) {
            return new Vector2d(-2.5, 2.9725);
        }
        return new Vector2d(-2.5, -2.9725);
    }
    // 51.5
    // 59
    // 66.5
    public static Vector2d[] powerShots(){
        if (blue) {
            return new Vector2d[]{
                    new Vector2d(70.4725, 3.9725),
                    new Vector2d(70.4725, 11.4725),
                    new Vector2d(70.4725, 18.9725)
            };
        }
        return new Vector2d[]{
                new Vector2d(70.4725, -3.9725),
                new Vector2d(70.4725, -11.4725),
                new Vector2d(70.4725, -18.9725)
        };
    }
    public static Pose2d[] dropZonesPS() {
        if (blue) {
            return new Pose2d[]{
                    new Pose2d(20, 47, Math.toRadians(120)),
                    new Pose2d(40, 20.4725, Math.toRadians(90)),
                    new Pose2d(57.5275, 47, Math.toRadians(60))
            };
        }
        return new Pose2d[]{
                new Pose2d(20, -47, Math.toRadians(-120)),
                new Pose2d(45, -20.4725, Math.toRadians(-90)),
                new Pose2d(57.5275, -47, Math.toRadians(-60))
        };

    }
}
