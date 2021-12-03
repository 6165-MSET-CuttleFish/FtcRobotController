package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Robot.*
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark

class AdvancedPaths {
    val capstone = Capstone()
    val deposit = Deposit()
    val intake = Intake()
    val carousel = Carousel()
    fun carouselPath(blue: Boolean): MeepMeep {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return MeepMeep(Robot.windowSize)
            .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY) // Set field image
            .setTheme(ColorSchemeRedDark()) // Set theme
            .setBackgroundAlpha(1f)
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                val trajectoryBuilder =
                    robot.trajectorySequenceBuilder(Robot.startingPosition())
                        .setReversed(true)
                        .capstoneReady(capstone)
                        .splineTo(
                            Robot.duckLocations()[0].vec(),
                            Math.toRadians(90.0).flip(blue) + Robot.duckLocations()[0].heading
                        )
                        .capstonePickup(capstone)
                        .liftUp(deposit)
                        .waitWhile(capstone::isDoingWork) // capstone loaded
                        .splineTo(Robot.dumpPosition().vec(), Robot.dumpPosition().heading)
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                        .splineTo(Vector2d(-55.0, -55.0).flip(blue), Math.toRadians(210.0).flip(blue))
                        .waitSeconds(1.5)
                        .carouselOff(carousel)// drop the ducky
                        .setReversed(true)
                        .splineTo(Vector2d(-24.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                        .turn(Math.toRadians(-180.0).flip(blue))
                        .setReversed(false)
                        .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
                        .splineTo(Vector2d(10.0, -30.0).flip(blue), Math.toRadians(-90.0).flip(blue))
                for (i in 1..3)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
                        }
                        .splineTo(Vector2d(20.0, -40.0), 0.0)
                        .splineTo(
                            Vector2d(39.0, -50.0).plus(
                                Vector2d(
                                    5 * Math.random(),
                                    5 * Math.random()
                                )
                            ).flip(blue), Math.toRadians(-35.0 + 10 * Math.random()).flip(blue)
                        )
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(Vector2d(20.0, -40.0), Math.toRadians(180.0).flip(blue))
                        .splineTo(Vector2d(9.0, -23.0).flip(blue), Math.toRadians(180.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                        .turn(Math.toRadians(-90.0).flip(blue))
                trajectoryBuilder
                    .splineTo(Vector2d(20.0, -40.0), 0.0)
                    .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
                    .build()
            }
    }

    fun cyclingPath(blue: Boolean): MeepMeep {
        side = Side.CYCLING
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
                        .splineTo(dumpPosition().vec(), Math.PI.flip(blue) + dumpPosition().heading)
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                for (i in 1..6)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
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
                        .intakeOff(intake)
                        .splineTo(cycleDumpPosition().vec(), cycleDumpPosition().heading + Math.PI.flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                trajectoryBuilder
                    .splineTo(Vector2d(45.0, -45.0).flip(blue), Math.toRadians(-35.0).flip(blue))
                    .build()
            }
    }
}