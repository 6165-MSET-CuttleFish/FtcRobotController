package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class PowerShots {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(1000)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(220), Math.toRadians(220), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-62, -22.7, 0))
                                .splineTo(new Vector2d(-5, -16), 0)
                                .waitSeconds(0.8) // Shoot powershots
                                .lineToLinearHeading(new Pose2d(59.5275, -10.7, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(59.5275, -50, Math.toRadians(-90)))
                                .turn(Math.toRadians(180))
                                .waitSeconds(1) // Drop Wobble
                                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bouncebacks
                                .lineTo(new Vector2d(12, -20))
                                .build()
                )
                .start();
    }
}
