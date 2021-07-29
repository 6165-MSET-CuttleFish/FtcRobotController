package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class BlueHighGoals {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(600)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark ())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(54, 54, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d (-62, 56, Math.toRadians(0)))

                                .lineTo(new Vector2d (2, 56))
                                .waitSeconds(0.1)
                                /*
                                //case 0
                                .lineToLinearHeading(new Pose2d(5, 52, Math.toRadians(270)))
                                .waitSeconds(0.8)
                                .lineTo(new Vector2d(5, 45))
                                .lineToLinearHeading (new Pose2d(-28, 54, Math.toRadians(0)))
                                .waitSeconds(12)
                                .lineTo(new Vector2d(-28, 38))
                                .lineTo(new Vector2d (8, 38))
                                */
                                /*
                                //case 1
                                .lineToLinearHeading(new Pose2d(39, 54, Math.toRadians(90)))
                                .waitSeconds(0.8)
                                .lineToLinearHeading(new Pose2d(-12,36, Math.toRadians(180)))
                                .lineTo(new Vector2d(-20, 36))
                                .waitSeconds (0.8)
                                .lineTo(new Vector2d(-4, 40))
                                .lineTo(new Vector2d(10, 40))

                                 */
                                /*
                                //case 4
                                .lineToLinearHeading(new Pose2d(49,52, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-12,36, Math.toRadians(180)))
                                .lineTo(new Vector2d(-20, 36))
                                .waitSeconds (0.8)
                                .lineTo(new Vector2d(-4, 36))
                                .waitSeconds(0.8)
                                .lineTo(new Vector2d(-30, 36))
                                .waitSeconds(0.2)
                                .lineTo(new Vector2d(-4, 40))
                                .waitSeconds(0.7)
                                .lineTo(new Vector2d(10, 40))
                                */

                                .build()
                )
                .start();

    }
}
