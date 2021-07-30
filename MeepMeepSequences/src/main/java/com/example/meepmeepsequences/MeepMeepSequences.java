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
                                drive.trajectorySequenceBuilder(new Pose2d(-61.5975, 16.8475, 0))
                                        .forward(10)
                                        .splineTo(new Vector2d(-2.5, 2.9725), 0)
                                        //wobble drop
//        wobbleDrop0 = robot.trajectorySequenceBuilder(powershots.end())
//                .lineToLinearHeading(new Pose2d(55.5275, 10.7, Math.toRadians(90)))
//                .splineTo(Robot.dropZonesPS()[0].vec(), Math.toRadians(90))
//                .turn(Math.toRadians(180))
//                .build();
//        wobbleDrop1 = robot.trajectorySequenceBuilder(powershots.end())
//                .lineToLinearHeading(new Pose2d(55.5275, 10.7, Math.toRadians(90)))
//                .splineTo(Robot.dropZonesPS()[1].vec(), Math.toRadians(90))
//                .turn(Math.toRadians(180))
//                .build();
                                        .lineToLinearHeading(new Pose2d(58.5275, 3, Math.toRadians(90)))
                                        .splineTo(new Vector2d(57.5275, 40), Math.toRadians(90))
                                        .turn(Math.toRadians(-180))
                                        .splineTo(new Vector2d(15, 20), Math.toRadians(180))
                                        .splineTo(new Vector2d(-5.8, 20), Math.toRadians(180))
                                        .lineTo(new Vector2d(12, 20))
                                        .build()
                )
                .start();

    }
}