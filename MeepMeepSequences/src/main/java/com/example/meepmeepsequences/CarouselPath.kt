package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Robot.*
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark

class CarouselPath {
    val capstone = Capstone()
    val lift = Deposit()
    fun carouselPath(blue: Boolean): MeepMeep {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return MeepMeep(windowSize)
                .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set field image
                .setTheme(ColorSchemeRedDark()) // Set theme
                .setBackgroundAlpha(1f)
                .configure() // configure robot
                .followTrajectorySequence { robot ->
                    val trajectoryBuilder =
                        robot.trajectorySequenceBuilder(startingPosition())
                            .setReversed(true)
                            .capstoneReady(capstone)
                            .splineTo(
                                duckLocations()[0].vec(),
                                Math.toRadians(90.0).flip(blue) + duckLocations()[0].heading
                            )
                            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                                println("Capstone Collected")
                                println("Lift Up")
                            }
                            .waitCondition { true } // duck loaded
                            .splineTo(cycleDump().vec(), cycleDump().heading)
                            .setReversed(false)
                            .waitCondition { true } // wait for platform to dump
                            .splineTo(Vector2d(-55.0, -55.0).flip(blue), Math.toRadians(210.0).flip(blue))
                            .waitSeconds(1.5) // drop the ducky
                            .setReversed(true)
                            .splineTo(Vector2d(-24.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                            .turn(Math.toRadians(-180.0).flip(blue))
                            .setReversed(false)
                            .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
                            .splineTo(Vector2d(10.0, -30.0).flip(blue), Math.toRadians(-90.0).flip(blue))
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
                                ).flip(blue), Math.toRadians(-35.0 + 10 * Math.random()).flip(blue)
                            )
                            .setReversed(true)
                            .splineTo(Vector2d(4.1, -34.0).flip(blue), Math.toRadians(150.0).flip(blue))
                            .waitCondition { true } // wait for platform to dump
                            .setReversed(false)
                    trajectoryBuilder
                        .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
                        .build()
                }
        }
}
