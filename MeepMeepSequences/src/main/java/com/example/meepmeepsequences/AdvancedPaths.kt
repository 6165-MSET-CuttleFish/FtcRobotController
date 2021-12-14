package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.Context.windowSize
import com.example.meepmeepsequences.util.FrequentPositions.allianceHub
import com.example.meepmeepsequences.util.FrequentPositions.cycleDumpPosition
import com.example.meepmeepsequences.util.FrequentPositions.duckLocation
import com.example.meepmeepsequences.util.FrequentPositions.dumpPosition
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.geometry.Line
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint

class AdvancedPaths {
    val capstone = Capstone()
    val deposit = Deposit()
    val intake = Intake()
    val carousel = Carousel()
    fun carouselPath(blue: Boolean): MeepMeep {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return MeepMeep(windowSize)
            .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY) // Set field image
            .setTheme(ColorSchemeRedDark()) // Set theme
            .setBackgroundAlpha(1f)
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                val trajectoryBuilder =
                    robot.trajectorySequenceBuilder(startingPosition())
                        .setReversed(true)
                        .capstoneReady(capstone)
                        .splineTo(
                            duckLocation().vec(),
                            Math.toRadians(90.0).flip(blue) + duckLocation().heading
                        )
                        .capstonePickup(capstone)
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .waitWhile(capstone::isDoingWork) // capstone loaded
                        .splineToCircle(allianceHub, Line.yAxis(-30.0).flip(blue), Vector2d(-20.0, -24.0).flip(blue))
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
                for (i in 1..4)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
                        }
                        .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                        .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                        .splineTo(
                            Vector2d(50.0, -45.0).plus(
                                Vector2d(
                                    5 * Math.random(),
                                )
                            ).flip(blue), Math.toRadians(-30.0 - 10 * Math.random()).flip(blue)
                        )
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(Vector2d(39.0, -40.0).flip(blue), Math.PI)
                        .splineToConstantHeading(Vector2d(20.0, -40.0).flip(blue), Math.PI)
                        .splineToCircle(allianceHub, Line.yAxis(-33.0), Vector2d(12.0, -24.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                trajectoryBuilder
                    .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
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
                        .setVelConstraint(getVelocityConstraint(30.0, 30.0, 15.0))
                        .splineTo(
                            duckLocation(Detector.Location.RIGHT).vec(),
                            Math.toRadians(90.0).flip(blue) + duckLocation().heading
                        )
                        .resetConstraints()
                        .capstonePickup(capstone)
                        .liftUp(deposit, Robot.getLevel(Detector.Location.RIGHT))
                        .waitWhile(capstone::isDoingWork) // capstone loaded
                        .splineTo(dumpPosition().vec(), Math.PI.flip(blue) + dumpPosition().heading)
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                for (i in 1..1)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
                        }
                        .splineTo(
                            Vector2d(20.0, -40.0).flip(blue), 0.0
                        )
                        .decreaseGains()
                        .splineTo(
                            Vector2d(24.0, -40.0).flip(blue), 0.0
                        )
                        .defaultGains()
                        .splineTo(
                            Vector2d(52.0, -50.0).plus(
                                Vector2d(
                                    5 * Math.random(),
                                    5 * Math.random()
                                )
                            ).flip(blue), Math.toRadians(-35.0 + 5 * Math.random()).flip(blue)
                        )
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(
                            Vector2d(26.0, -40.0).flip(blue), Math.PI
                        )
                        .decreaseGains()
                        .splineTo(
                            Vector2d(24.0, -40.0).flip(blue), Math.PI
                        )
                        .defaultGains()
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