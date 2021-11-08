package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.Alliance
import com.example.meepmeepsequences.util.Robot.*
import com.example.meepmeepsequences.util.Side
import com.example.meepmeepsequences.util.waitCondition
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DriveTrainType

class CarouselPath {
    val carouselPathRed: MeepMeep
        get() {
            side = Side.CAROUSEL
            alliance = Alliance.RED
            return MeepMeep(800)
                .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeRedDark()) // Set theme
                .setDriveTrainType(DriveTrainType.TANK)
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80.0, 80.0, Math.toRadians(720.0), Math.toRadians(720.0), 15.0)
                .followTrajectorySequence { robot ->
                    var trajectoryBuilder = robot.trajectorySequenceBuilder(Pose2d(-36.0, -58.0, Math.toRadians(-90.0)))
                        .setReversed(true)
                        .splineTo(duckLocations()[0].vec(), Math.toRadians(90.0) + duckLocations()[0].heading)
                        .waitCondition { true } // duck loaded
                        .splineTo(Vector2d(-28.0, -31.0), Math.toRadians(20.0))
                        .setReversed(false)
                        .waitCondition { true } // wait for platform to dump
                        .splineTo(Vector2d(-55.0, -55.0), Math.toRadians(210.0))
                        .waitSeconds(1.5) // drop the ducky
                        .setReversed(true)
                        .splineTo(Vector2d(-24.0, -4.0), Math.toRadians(0.0))
                        .turn(Math.toRadians(-180.0))
                        .setReversed(false)
                        .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)))
                        .splineTo(Vector2d(10.0, -15.0), Math.toRadians(-90.0))
                    for (i in 1..5) {
                        trajectoryBuilder = trajectoryBuilder
                            .splineTo(Vector2d(45.0, -45.0), Math.toRadians(-30.0))
                            .setReversed(true)
                            .splineTo(Vector2d(4.1, -34.0), Math.toRadians(150.0))
                            .waitCondition { true } // wait for platform to dump
                            .setReversed(false)
                    }
                    trajectoryBuilder
                        .build()
                }
        }
    val carouselPathBlue: MeepMeep
        get() {
            side = Side.CAROUSEL
            alliance = Alliance.BLUE
            return MeepMeep(800)
                .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeBlueDark()) // Set theme
                .setDriveTrainType(DriveTrainType.TANK)
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80.0, 80.0, Math.toRadians(720.0), Math.toRadians(720.0), 15.0)
                .followTrajectorySequence { robot ->
                    var trajectoryBuilder = robot.trajectorySequenceBuilder(Pose2d(-36.0, -58.0, Math.toRadians(-90.0)))
                        .setReversed(true)
                        .splineTo(duckLocations()[2].vec(), Math.toRadians(90.0) + duckLocations()[0].heading)
                        .waitCondition { true } // duck loaded
                        .splineTo(Vector2d(-28.0, -31.0), Math.toRadians(20.0))
                        .setReversed(false)
                        .splineTo(Vector2d(-55.0, -55.0), Math.toRadians(210.0))
                        .waitSeconds(1.5) // drop the ducky
                        .setReversed(true)
                        .splineTo(Vector2d(-24.0, -4.0), Math.toRadians(0.0))
                        .turn(Math.toRadians(-180.0))
                        .setReversed(false)
                        .splineTo(Vector2d(6.0, -15.0), Math.toRadians(-90.0))
                    for (i in 1..3) {
                        trajectoryBuilder = trajectoryBuilder
                            .splineTo(Vector2d(45.0, -45.0), Math.toRadians(-30.0))
                            .setReversed(true)
                            .splineTo(Vector2d(3.0, -30.0), Math.toRadians(150.0))
                            .setReversed(false)
                    }
                    trajectoryBuilder
                        .build()
                }
        }
}
