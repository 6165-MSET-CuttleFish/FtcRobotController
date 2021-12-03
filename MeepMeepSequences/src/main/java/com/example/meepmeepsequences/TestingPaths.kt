package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark

class TestingPaths {
    val capstone = Capstone()
    val deposit = Deposit()
    val intake = Intake()
    val carousel = Carousel()
    fun waitWhileTest(blue: Boolean): MeepMeep {
        Robot.side = Side.CAROUSEL
        Robot.alliance = if (blue) Alliance.BLUE else Alliance.RED
        return MeepMeep(Robot.windowSize)
            .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY) // Set field image
            .setTheme(ColorSchemeRedDark()) // Set theme
            .setBackgroundAlpha(1f)
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                    robot.trajectorySequenceBuilder(Pose2d())
                        .splineTo(Vector2d(30.0, 30.0), 0.0)
                        .UNSTABLE_addTemporalMarkerOffset(0.0, deposit::dump)
                        .waitWhile(deposit::isDoingWork)
                        .setReversed(true)
                        .splineTo(Vector2d(), Math.PI)
                        .build()
            }
    }
}