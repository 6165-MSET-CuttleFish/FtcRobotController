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
                                drive.trajectorySequenceBuilder(new Pose2d(-61.5975, -16.8475, 0))
                                .lineToSplineHeading(new Pose2d(46, -16))
                                .splineTo(new Vector2d(58, -10), Math.toRadians(90))
                                .splineTo(new Vector2d(58, 17), Math.toRadians(90))
        // Wobble Drop
//        wobbleDrop0 = robot.trajectoryBuilder(mainSequence.end(), true)
//                .splineTo(new Vector2d(50.5275, -5), Math.toRadians(-90))
//                .splineTo(Robot.dropZonesPS()[0].vec(), Robot.dropZonesPS()[0].getHeading())
//                .build();
//        wobbleDrop1 = robot.trajectoryBuilder(mainSequence.end(), true)
//                .splineTo(new Vector2d(50.5275, -5), Math.toRadians(-90))
//                .splineTo(Robot.dropZonesPS()[1].vec(), Robot.dropZonesPS()[1].getHeading())
//                .build();
//        wobbleDrop4 = robot.trajectoryBuilder(mainSequence.end(), true)
                                        .setReversed(true)
                .splineTo(new Vector2d(58, -5), Math.toRadians(-90))
                .splineTo(Robot.dropZonesPS()[2].vec(), Robot.dropZonesPS()[2].getHeading())
                                        .setReversed(false)
        // Shoot Bonked
                                        .splineTo(new Vector2d(10, -16.7), Math.toRadians(180))
                .splineTo(new Vector2d(-5, -16.7), Math.toRadians(180))
                .lineTo(new Vector2d(-55, -16.7)) // Intake starter rings
                .setReversed(true)
                .splineTo(Robot.pwrShotLocal(), 0)
                .lineToLinearHeading(new Pose2d(58, -3, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(58.5275, -40, Math.toRadians(-90)))
                .setReversed(true)

                                        .splineTo(new Vector2d(20, -16), Math.toRadians(180))
                .splineTo(new Vector2d(-5.8, -17), Math.toRadians(180))
                .lineTo(new Vector2d(12, -17))
                .build()
                )
                .start();

    }
}