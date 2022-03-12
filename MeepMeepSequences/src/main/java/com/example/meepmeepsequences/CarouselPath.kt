package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.FrequentPositions
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
                    .splineTo(FrequentPositions.allianceHub.center.polarAdd(20.0,Math.toRadians(-100.0).flip(blue)), FrequentPositions.allianceHub.center)
                    .waitSeconds(0.5)
                    .waitWhile(deposit::isDoingWork)
                    .setReversed(false)
                    .forward(7.0)
                    .turn(Math.toRadians(240.0).flip(blue))
                    .splineTo(Vector2d(-50.0,-46.0).flip(blue),Math.toRadians(175.0).flip(blue))
                    .setReversed(true)
                    .splineTo(FrequentPositions.carouselVec.center.polarAdd(13.0, Math.toRadians(45.0).flip(blue)), FrequentPositions.carouselVec.center)
                    .waitSeconds(2.0)
                    .setReversed(false)
                    .carouselOff(carousel)
                    .resetConstraints()
                    .splineTo(Vector2d(-56.0, -24.0).flip(blue), Math.toRadians(90.0).flip(blue))
                    .build()
            }
    }
}