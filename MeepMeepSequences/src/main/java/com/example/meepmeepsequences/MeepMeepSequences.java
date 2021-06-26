package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepSequences {
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
                                .splineTo(new Vector2d(50.5275, -22.7), Math.toRadians(0))
                                .splineTo(new Vector2d(65.5275, -22.7), Math.toRadians(90))
                                .splineTo(new Vector2d(67, 10), Math.toRadians(90))
                                .splineTo(new Vector2d(63.5, 20), Math.toRadians(90))
                                .splineTo(new Vector2d(20, 20), Math.toRadians(180))
                                .splineTo(new Vector2d(-5, -16), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot Boinked Rings
                                .lineTo(new Vector2d(-62, -22.7)) // Intake starter rings
                                .lineToLinearHeading(new Pose2d(-5, -16))
                                .waitSeconds(0.8) // Shoot powershots
                                .lineToLinearHeading(new Pose2d(65.5275, -10.7, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(60.5275, -57, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(60.5275, -50, Math.toRadians(90)))
                                .waitSeconds(1) // Drop Wobble
                                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bouncebacks
                                .lineTo(new Vector2d(12, -20))
                                .build()
                )
                .start();
    }

    public static void powerShots(String[] args) {
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
                                .splineTo(new Vector2d(50.5275, -22.7), Math.toRadians(0))
                                .splineTo(new Vector2d(65.5275, -22.7), Math.toRadians(90))
                                .splineTo(new Vector2d(67, 10), Math.toRadians(90))
                                .splineTo(new Vector2d(63.5, 20), Math.toRadians(90))
                                .splineTo(new Vector2d(20, 20), Math.toRadians(180))
                                .splineTo(new Vector2d(-5, -16), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot Boinked Rings
                                .lineTo(new Vector2d(-62, -22.7)) // Intake starter rings
                                .lineToLinearHeading(new Pose2d(-5, -16))
                                .waitSeconds(0.8) // Shoot powershots
                                .lineToLinearHeading(new Pose2d(65.5275, -10.7, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(60.5275, -57, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(60.5275, -50, Math.toRadians(90)))
                                .waitSeconds(1) // Drop Wobble
                                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bouncebacks
                                .lineTo(new Vector2d(12, -20))
                                .build()
                )
                .start();
    }
}