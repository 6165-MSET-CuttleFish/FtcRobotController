package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.Robot.duckLocations
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import java.util.*;

class Path {

    // Declare a MeepMeep instance
    // With a field size of 800 pixels
    val aggroPath = MeepMeep(800) // Set field image
        .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set theme
        .setTheme(ColorSchemeRedDark())
        .setDriveTrainType(DriveTrainType.TANK)
        // Background opacity from 0-1
        .setBackgroundAlpha(1f) // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(80.0, 80.0, Math.toRadians(200.0), Math.toRadians(200.0), 15.0)
        .followTrajectorySequence { drive ->
            drive.trajectorySequenceBuilder(Pose2d(11.0, -60.0, Math.toRadians(-90.0)))
                .setReversed(true)
                .splineTo(duckLocations()[0], Math.toRadians(90.0))
                .splineTo(Vector2d(-3.0, -35.0), Math.toRadians(120.0))
                .setReversed(false)
                .splineTo(Vector2d(50.0, -50.0), Math.toRadians(-60.0))
                .setReversed(true)
                .splineTo(Vector2d(0.0, -28.0), Math.toRadians(160.0))
                .build()
        }
}
