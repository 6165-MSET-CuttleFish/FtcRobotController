package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Details.alliance
import com.example.meepmeepsequences.util.Details.location
import com.example.meepmeepsequences.util.Details.side
import com.example.meepmeepsequences.util.Details.windowSize
import com.example.meepmeepsequences.util.FrequentPositions.duckLocations
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.Robot.getLevel
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint

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
                            .liftUp(deposit, getLevel(location))
                            .splineTo(Vector2d(-11.0, -42.0).flip(blue), Math.toRadians(90.0).flip(blue))
                            .setReversed(false)
                            .dump(deposit)
                            .waitWhile(deposit::isDoingWork) // wait for platform to dump
//                            .UNSTABLE_addTemporalMarkerOffset(0.0) {
//                                admissibleError = Pose2d(5.0, 5.0, Math.toRadians(20.0))
//                            }
                            .setVelConstraint(getVelocityConstraint(10.0, Math.PI,15.0))
                            // .splineTo(Vector2d(-45.5, -45.5).flip(blue), Math.toRadians(215.0).flip(blue))
                            // .setVelConstraint(getVelocityConstraint(5.0, Math.PI,15.0))
                            .UNSTABLE_addTemporalMarkerOffset(0.1, carousel::on)
//                            .UNSTABLE_addTemporalMarkerOffset(1.0) {
//                                admissibleError = Pose2d(2.0, 2.0, Math.toRadians(5.0))
//                            }
                            .splineTo(Vector2d(-58.0, -53.0).flip(blue), Math.toRadians(203.0).flip(blue))
                            .waitSeconds(3.0)
                            .carouselOff(carousel)// drop the ducky
                            .resetConstraints()
                            .setReversed(true)
                            .splineTo(Vector2d(-61.0, -35.0).flip(blue), Math.toRadians(180.0).flip(blue))
                    trajectoryBuilder
                        .build()
                }
        }
}
