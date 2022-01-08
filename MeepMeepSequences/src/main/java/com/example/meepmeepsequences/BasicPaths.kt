package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.Robot.getLevel
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

class BasicPaths {
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
                            .back(8.0)
                            //.setConstraints(getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(40.0))
                            .turn(Math.toRadians(90.0).flip(blue))
                            .splineTo(Vector2d(-55.0, -24.8).flip(blue), Math.toRadians(-270.0).flip(blue))
                            //.setConstraints(getVelocityConstraint(15.0, DriveConstants.MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(15.0))
                            .turn(Math.toRadians(-90.0).flip(blue))
                            .capstoneReady(capstone)
                            .waitWhile { capstone.isDoingWork() }
                            .back(20.0)
                            .resetConstraints()
                            .capstonePickup(capstone)
                            .liftUp(deposit, getLevel(location))
                            .waitWhile(capstone::isDoingWork) // capstone loaded
                            .splineTo(Vector2d(-30.0, -25.0).flip(blue), Math.toRadians(0.0).flip(blue))
                            .setReversed(false)
                            .dump(deposit)
                            .waitWhile(deposit::isDoingWork) // wait for platform to dump
//                            .UNSTABLE_addTemporalMarkerOffset(0.0) {
//                                admissibleError = Pose2d(5.0, 5.0, Math.toRadians(20.0))
//                            }
                            .setVelConstraint(getVelocityConstraint(10.0, Math.PI,15.0))
                            // .splineTo(Vector2d(-45.5, -45.5).flip(blue), Math.toRadians(215.0).flip(blue))
                            // .setVelConstraint(getVelocityConstraint(5.0, Math.PI,15.0))
//                            .UNSTABLE_addTemporalMarkerOffset(1.0) {
//                                admissibleError = Pose2d(2.0, 2.0, Math.toRadians(5.0))
//                            }
                            .splineTo(Vector2d(-58.0, -53.0).flip(blue), Math.toRadians(203.0).flip(blue))
                            .UNSTABLE_addTemporalMarkerOffset(-0.5, carousel::on)
                            .waitSeconds(3.0)
                            .carouselOff(carousel)// drop the ducky
                            .resetConstraints()
                            .setReversed(true)
                            .splineTo(Vector2d(-61.0, -34.0).flip(blue), Math.toRadians(180.0).flip(blue))
                   trajectoryBuilder
                        .build()
                }
        }
}
