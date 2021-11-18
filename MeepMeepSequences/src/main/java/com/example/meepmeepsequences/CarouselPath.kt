package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Robot.*
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark

class CarouselPath {
    val carouselPathRed: MeepMeep
        get() {
            side = Side.CAROUSEL
            alliance = Alliance.RED
            return MeepMeep(windowSize)
                .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeRedDark()) // Set theme
                .setBackgroundAlpha(1f)
                .configure() // configure robot
                .followTrajectorySequence { robot ->
                    val trajectoryBuilder =
                        robot.trajectorySequenceBuilder(startingPosition())
                            .setReversed(true)
                            .splineTo(
                                duckLocations()[0].vec(),
                                Math.toRadians(90.0) + duckLocations()[0].heading
                            )
                            .waitCondition { true } // duck loaded
                            .splineTo(cycleDump().vec(), cycleDump().heading)
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
                    for (i in 1..5)
                        trajectoryBuilder
                            .splineTo(
                                Vector2d(39.0, -50.0).plus(
                                    Vector2d(
                                        5.0,
                                        5.0.random()
                                    )
                                ), Math.toRadians(-35.0 + 5.0.random())
                            )
                            .setReversed(true)
                            .splineTo(Vector2d(4.1, -34.0), Math.toRadians(150.0))
                            .waitCondition { true } // wait for platform to dump
                            .setReversed(false)
                    trajectoryBuilder
                        .splineTo(Vector2d(45.0, -45.0), Math.toRadians(-35.0))
                        .build()
                }
        }

    val carouselPathBlue: MeepMeep
        get() {
            side = Side.CAROUSEL
            alliance = Alliance.BLUE
            return MeepMeep(windowSize)
                .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeRedDark()) // Set theme
                .setBackgroundAlpha(1f)
                .configure() // configure robot
                .followTrajectorySequence { robot ->
                    val trajectoryBuilder =
                        robot.trajectorySequenceBuilder(startingPosition())
                            .setReversed(true)
                            .capstoneReady(Capstone())
                            .splineTo(
                                duckLocations()[0].vec(),
                                Math.toRadians(-90.0) + duckLocations()[0].heading
                            )
                            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                                println("Capstone Collected")
                                println("Lift Up")
                            }
                            .waitCondition { true } // duck loaded
                            .splineTo(cycleDump().vec(), cycleDump().heading)
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
                    for (i in 1..5)
                        trajectoryBuilder
                            .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                                println("Intake Extended")
                            }
                            .splineTo(
                                Vector2d(39.0, -50.0).plus(
                                    Vector2d(
                                        5 * Math.random(),
                                        5 * Math.random()
                                    )
                                ).flip(true), Math.toRadians(-35.0 + 10 * Math.random()).flip(true)
                            )
                            .setReversed(true)
                            .splineTo(Vector2d(4.1, -34.0).flip(true), Math.toRadians(150.0).flip(true))
                            .waitCondition { true } // wait for platform to dump
                            .setReversed(false)
                    trajectoryBuilder
                        .splineTo(Vector2d(45.0, -45.0).flip(true), Math.toRadians(-35.0).flip(true))
                        .build()
                }
        }
}
