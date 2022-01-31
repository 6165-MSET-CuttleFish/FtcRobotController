package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.FrequentPositions.allianceHub
import com.example.meepmeepsequences.util.FrequentPositions.barcode
import com.example.meepmeepsequences.util.FrequentPositions.carouselVec
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.geometry.Line
import com.example.meepmeepsequences.util.geometry.flip
import com.example.meepmeepsequences.util.geometry.polarAdd
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

class CarouselPath {
    private fun colorSchemeVariable() = if (alliance == Alliance.BLUE) ColorSchemeBlueDark() else ColorSchemeRedDark()
    private val capstone = Capstone()
    private val deposit = Deposit()
    private val intake = Intake()
    private val carousel = Carousel()
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
                trajectoryBuilder
                    .splineToCircle(allianceHub, Line.yAxis(-45.0).flip(blue), Vector2d(-14.0, -43.0).flip(blue))
                    .waitWhile(deposit::isDoingWork)
                    .setReversed(false)
                    .splineTo(carouselVec.center.polarAdd(carouselVec.radius, Math.toRadians(40.0).flip(blue)), carouselVec.center)
                    .waitWhile(carousel::isDoingWork)
                    .setReversed(true)
                    .splineTo(Vector2d(-59.0, -35.0).flip(blue), Math.toRadians(90.0).flip(blue))
                    .build()
            }
    }
}
/*
        .setReversed(true)
                    .splineTo(Vector2d(-53.0, -50.0).flip(blue), Math.toRadians(0.0).flip(blue))
                    .splineTo(Vector2d(-30.0, -57.0).flip(blue), Math.toRadians(0.0).flip(blue))
                    .splineToConstantHeading(Vector2d(39.0, -57.0).flip(blue), 0.0)
                    */