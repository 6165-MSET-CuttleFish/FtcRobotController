package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Robot.*
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
                            .splineTo(Vector2d(-26.0, -36.0).flip(blue), Math.toRadians(40.0).flip(blue))
                            .setReversed(false)
                            .dump(deposit)
                            .waitWhile(deposit::isDoingWork) // wait for platform to dump
                            .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                            .splineTo(Vector2d(-55.0, -55.0).flip(blue), Math.toRadians(210.0).flip(blue))
                            .waitSeconds(1.5)
                            .carouselOff(carousel)// drop the ducky
                            .setReversed(true)
                            .splineTo(Vector2d(-60.0, -36.0).flip(blue), Math.toRadians(90.0).flip(blue))
                    trajectoryBuilder

                        .build()
                }
        }
}
