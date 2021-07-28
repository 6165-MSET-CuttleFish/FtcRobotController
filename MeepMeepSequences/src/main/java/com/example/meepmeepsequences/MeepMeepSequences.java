package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepSequences {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(600)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(54, 54, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-64, -64, Math.toRadians(0)))
                                .lineToLinearHeading(new Vector2d(-40, -64), Math.toRadians(180)))

                                .build()
                )
                .start();
                        /*
                        drive.trajectorySequenceBuilder(new Pose2d(-62, -22.7, 0))
                                .splineTo(new Vector2d(50.5275, -22.7), Math.toRadians(0))
                                .splineTo(new Vector2d(60.5275, -22.7), Math.toRadians(90))
                                .splineTo(new Vector2d(60.5, 20), Math.toRadians(90))
                                .setReversed(true)
                                .splineTo(new Vector2d(60.5275, -50), Math.toRadians(-90)) //!!!
                                .waitSeconds(0.5) // Drop Wobble
                                .setReversed(false)
                                .splineTo(new Vector2d(-5, -22.7), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bonked rings
                                .lineTo(new Vector2d(-62, -22.7)) // Intake starter rings
                                .setReversed(true)
                                .splineTo(new Vector2d(-5, -10.7), Math.toRadians(0))
                                .waitSeconds(0.6) // Shoot powershots
                                .lineToLinearHeading(new Pose2d(65.5275, -10.7, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(60.5275, -57, Math.toRadians(-90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bouncebacks
                                .lineTo(new Vector2d(12, -20))
                                .build()

                         */

                // reversed for blue

                            /*
                                .splineTo(new Vector2d(50.5275, 22.7), Math.toRadians(0))
                                .splineTo(new Vector2d(60.5275, 22.7), Math.toRadians(270))
                                .splineTo(new Vector2d(60.5, -20), Math.toRadians(270))
                                .setReversed(true)
                                .splineTo(new Vector2d(60.5275, 50), Math.toRadians(90)) //!!!
                                .waitSeconds(0.5) // Drop Wobble
                                .setReversed(false)
                                .splineTo(new Vector2d(-5, 22.7), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bonked rings
                                .lineTo(new Vector2d(-62, 22.7)) // Intake starter rings
                                .setReversed(true)
                                .splineTo(new Vector2d(-5, 10.7), Math.toRadians(0))
                                .waitSeconds(0.6) // Shoot powershots
                                .lineToLinearHeading(new Pose2d(60.5275, 10.7, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(60.5275, 57, Math.toRadians(90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-5.8, 20), Math.toRadians(180))
                                .waitSeconds(0.5) // Shoot bouncebacks
                                .lineTo(new Vector2d(12, 20))
                                .build()

                        */

    }
}