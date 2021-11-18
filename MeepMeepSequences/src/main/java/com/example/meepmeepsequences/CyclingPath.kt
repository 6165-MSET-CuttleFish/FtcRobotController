package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Robot.windowSize
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark

class CyclingPath {
    val carouselPathRed: MeepMeep
        get() {
            Robot.side = Side.CAROUSEL
            Robot.alliance = Alliance.RED
            return MeepMeep(windowSize)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeRedDark()) // Set theme
                .setBackgroundAlpha(1f)
                .configure() // configure robot
                .followTrajectorySequence { robot ->
                    var trajectoryBuilder =
                        robot.trajectorySequenceBuilder(Robot.startingPosition())
                            .setReversed(true)
                            .splineTo(
                                Robot.duckLocations()[0].vec(),
                                Math.toRadians(90.0) + Robot.duckLocations()[0].heading
                            )
                            .waitCondition { true } // duck loaded
                            .splineTo(Robot.cycleDump().vec(), Robot.cycleDump().heading)
                            .setReversed(false)
                            .waitCondition { true } // wait for platform to dump
                            .splineTo(Vector2d(-55.0, -55.0), Math.toRadians(210.0))
                            .waitSeconds(1.5) // drop the ducky
                            .setReversed(true)
                            .splineTo(Vector2d(-24.0, -4.0), Math.toRadians(0.0))
                            .turn(Math.toRadians(-180.0))
                            .setReversed(false)
                            .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)))
                            .splineTo(Vector2d(10.0, -30.0), Math.toRadians(-90.0))
                    for (i in 1..5) {
                        trajectoryBuilder = trajectoryBuilder
                            .splineTo(
                                Vector2d(39.0, -50.0).plus(
                                    Vector2d(
                                        5 * Math.random(),
                                        5 * Math.random()
                                    )
                                ), Math.toRadians(-35.0 + 5 * Math.random())
                            )
                            .setReversed(true)
                            .splineTo(Vector2d(4.1, -34.0), Math.toRadians(150.0))
                            .waitCondition { true } // wait for platform to dump
                            .setReversed(false)
                    }
                    trajectoryBuilder
                        .splineTo(Vector2d(45.0, -45.0), Math.toRadians(-35.0))
                        .build()
                }
        }

    val carouselPathBlue: MeepMeep
        get() {
            Robot.side = Side.CAROUSEL
            Robot.alliance = Alliance.BLUE
            return MeepMeep(windowSize)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeRedDark()) // Set theme
                .setBackgroundAlpha(1f)
                .configure() // configure robot
                .followTrajectorySequence { robot ->
                    var trajectoryBuilder =
                        robot.trajectorySequenceBuilder(Robot.startingPosition())
                            .setReversed(true)
                            .splineTo(
                                Robot.duckLocations()[0].vec(),
                                Math.toRadians(-90.0) + Robot.duckLocations()[0].heading
                            )
                            .waitCondition { true } // duck loaded
                            .splineTo(Robot.cycleDump().vec(), Robot.cycleDump().heading)
                            .setReversed(false)
                            .waitCondition { true } // wait for platform to dump
                            .splineTo(Vector2d(-55.0, -55.0).flip(true), Math.toRadians(210.0).flip(true))
                            .waitSeconds(1.5) // drop the ducky
                            .setReversed(true)
                            .splineTo(Vector2d(-24.0, -4.0).flip(true), Math.toRadians(0.0).flip(true))
                            .turn(Math.toRadians(-180.0).flip(true))
                            .setReversed(false)
                            .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(true))
                            .splineTo(Vector2d(10.0, -30.0).flip(true), Math.toRadians(-90.0).flip(true))
                    for (i in 1..5) {
                        trajectoryBuilder = trajectoryBuilder
                            .splineTo(
                                Vector2d(39.0, -50.0).plus(
                                    Vector2d(
                                        5 * Math.random(),
                                        5 * Math.random()
                                    )
                                ).flip(true), Math.toRadians(-35.0 + 5.0.random()).flip(true)
                            )
                            .setReversed(true)
                            .splineTo(Vector2d(4.1, -34.0).flip(true), Math.toRadians(150.0).flip(true))
                            .waitCondition { true } // wait for platform to dump
                            .setReversed(false)
                    }
                    trajectoryBuilder
                        .splineTo(Vector2d(45.0, -45.0).flip(true), Math.toRadians(-35.0).flip(true))
                        .build()
                }
        }
}