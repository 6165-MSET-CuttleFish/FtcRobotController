package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Details.alliance
import com.example.meepmeepsequences.util.Details.side
import com.example.meepmeepsequences.util.Details.windowSize
import com.example.meepmeepsequences.util.FrequentPositions.duckLocations
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark

class BasicCarouselPath {
    val capstone = Capstone()
    val deposit = Deposit()
    val intake = Intake()
    val carousel = Carousel()
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
                            .capstonePickup(capstone)
                            .liftUp(deposit)
                            .waitWhile(capstone::isDoingWork) // capstone loaded
                            .splineTo(Vector2d(-30.0, -34.0).flip(blue), Math.toRadians(30.0).flip(blue))
                            .setReversed(false)
                            .dump(deposit)
                            .waitWhile(deposit::isDoingWork) // wait for platform to dump
                            .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                            .splineTo(Vector2d(-50.0, -50.0).flip(blue), Math.toRadians(215.0).flip(blue))
                            .forward(6.0)
                            .waitSeconds(1.5)
                            .carouselOff(carousel)// drop the ducky
                            .back(6.0)
                            .setReversed(true)
                            .splineTo(Vector2d(-63.0, -35.0).flip(blue), Math.toRadians(180.0).flip(blue))
                    trajectoryBuilder

                        .build()
                }
        }
}
