package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.FrequentPositions.allianceHub
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.Robot.getLevel
import com.example.meepmeepsequences.util.geometry.flip
import com.example.meepmeepsequences.util.geometry.polarAdd
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

class BasicPaths {
    private fun colorSchemeVariable() = if (alliance == Alliance.BLUE) ColorSchemeBlueDark() else ColorSchemeRedDark()
    private val capstone = Capstone()
    private val deposit = Deposit()
    val intake = Intake()
    private val carousel = Carousel()
    fun carouselPath(blue: Boolean, meepMeep: MeepMeep): RoadRunnerBotEntity {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return DefaultBotBuilder(meepMeep)
            .setColorScheme(colorSchemeVariable()) // Set theme
            .configure() // configure robot
                .followTrajectorySequence { robot ->
                        robot.trajectorySequenceBuilder(startingPosition())
                            .back(7.0)
                            .turn(Math.toRadians(90.0).flip(blue))
                            .setReversed(true)
                            .setVelConstraint(getVelocityConstraint(20.0, Math.PI,15.0))
                            .splineTo(Vector2d(-58.0, -53.0).flip(blue), Math.toRadians(220.0).flip(blue))
                            .waitSeconds(2.0, DriveSignal(Pose2d(-5.0, 0.0, Math.toRadians(0.0))))
                            .resetConstraints()
                            .setReversed(false)
                            .splineTo(Vector2d(-58.0, -27.0).flip(blue), Math.toRadians(90.0).flip(blue))
                            .splineTo(Vector2d(-39.0, -8.0).flip(blue), Math.toRadians(120.0).flip(blue))
                            .setReversed(true)
                            .splineTo(allianceHub.center.polarAdd(20.0, Math.toRadians(160.0).flip(blue)), allianceHub.center)
                            .setReversed(false)
                            .splineTo(Vector2d(-58.0, -16.0).flip(blue), Math.toRadians(180.0).flip(blue))
                            .turn(Math.toRadians(90.0))
                            .splineTo(Vector2d(-58.0, -35.0).flip(blue), Math.toRadians(-90.0).flip(blue))
                            .build()
                }
        }
}
