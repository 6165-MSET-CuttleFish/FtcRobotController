package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.Context.windowSize
import com.example.meepmeepsequences.util.FrequentPositions.allianceHub
import com.example.meepmeepsequences.util.FrequentPositions.barcode
import com.example.meepmeepsequences.util.FrequentPositions.carouselVec
import com.example.meepmeepsequences.util.FrequentPositions.cycleDumpPosition
import com.example.meepmeepsequences.util.FrequentPositions.duckLocation
import com.example.meepmeepsequences.util.FrequentPositions.dumpPosition
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.geometry.Line
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

class AdvancedPaths {
    fun colorSchemeVariable() = if (alliance == Alliance.BLUE) ColorSchemeBlueDark() else ColorSchemeRedDark()
    val capstone = Capstone()
    val deposit = Deposit()
    val intake = Intake()
    val carousel = Carousel()
    fun carouselPath(blue: Boolean, meepMeep: MeepMeep): RoadRunnerBotEntity {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return DefaultBotBuilder(meepMeep)
            .setColorScheme(colorSchemeVariable()) // Set theme
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                val trajectoryBuilder =
                    robot.trajectorySequenceBuilder(startingPosition())
                        .setReversed(true)
                        .capstoneReady(capstone)
                        .splineToVectorOffset(barcode[1].vec().flip(blue), Pose2d(14.0, -8.0), (Math.PI / 2 + barcode[1].heading).flip(blue))
                        .capstonePickup(capstone)
                        .liftUp(deposit, Robot.getLevel(location))
                        .waitWhile(capstone::isDoingWork) // capstone loaded
                        .splineToCircle(allianceHub, Line.yAxis(-33.0).flip(blue), Vector2d(-20.0, -24.0).flip(blue))
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                        .splineTo(carouselVec.center.polarAdd(carouselVec.radius, Math.toRadians(40.0).flip(blue)), carouselVec.center)
                        .waitSeconds(1.2, DriveSignal(Pose2d(5.0), Pose2d(5.0)))
                        .carouselOff(carousel) // drop the ducky
                        .turn(Math.toRadians(45.0))
                        .intakeOff(intake)
                        .setReversed(true)
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineToCircle(allianceHub, Line.yAxis(-19.0).flip(blue), Vector2d(-23.0, -10.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork)
                        .setReversed(false)
                        .forward(3.0)
                        .splineTo(Vector2d(-12.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                        .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
                        .splineTo(Vector2d(10.0, -30.0).flip(blue), Math.toRadians(-90.0).flip(blue))
                for (i in 1..4)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
                        }
                        .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                        .increaseGains()
                        .splineToConstantHeading(Vector2d(28.0, -40.0).flip(blue), 0.0)
                        .defaultGains()
                        .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                        .splineTo(Vector2d(50.0, -45.0).flip(blue), Math.toRadians(-30.0 - 20 * Math.random()).flip(blue))
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(Vector2d(39.0, -40.0).flip(blue), Math.PI)
                        .increaseGains()
                        .splineToConstantHeading(Vector2d(20.0, -40.0).flip(blue), Math.PI)
                        .defaultGains()
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineToCircle(allianceHub, Line.yAxis(-33.0).flip(blue), Vector2d(12.0, -24.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                trajectoryBuilder
                    .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                    .build()
            }
    }

    fun cyclingPath(blue: Boolean, meepMeep: MeepMeep): RoadRunnerBotEntity {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return DefaultBotBuilder(meepMeep)
            .setColorScheme(colorSchemeVariable()) // Set theme
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                val trajectoryBuilder =
                    robot.trajectorySequenceBuilder(startingPosition())
                        .setReversed(true)
                        .capstoneReady(capstone)
                        .splineToVectorOffset(barcode[1].vec().flip(blue), Pose2d(14.0, -8.0), (Math.PI / 2 + barcode[1].heading).flip(blue))
                        .capstonePickup(capstone)
                        .liftUp(deposit, Robot.getLevel(location))
                        .waitWhile(capstone::isDoingWork) // capstone loaded
                        .splineToCircle(allianceHub, Line.yAxis(-33.0).flip(blue), Vector2d(1.0, -30.0).flip(blue))
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                for (i in 1..7)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
                        }
                        .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                        //.increaseGains()
                        .splineToConstantHeading(Vector2d(28.0, -40.0).flip(blue), 0.0)
                        .defaultGains()
                        .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                        .splineTo(Vector2d(50.0 + 5 * Math.random(), -45.0).flip(blue), Math.toRadians(-30.0 - 20 * Math.random()).flip(blue))
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(Vector2d(39.0, -40.0).flip(blue), Math.PI)
                        //.increaseGains()
                        .splineToConstantHeading(Vector2d(20.0, -40.0).flip(blue), Math.PI)
                        .defaultGains()
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineToCircle(allianceHub, Line.yAxis(-33.0 - 0.5*i.toDouble().flip(blue)).flip(blue), Vector2d(12.0, -24.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                trajectoryBuilder
                    .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                    .build()
            }
    }
}