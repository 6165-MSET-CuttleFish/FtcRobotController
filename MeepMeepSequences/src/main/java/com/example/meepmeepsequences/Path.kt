package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.example.meepmeepsequences.Robot.dropZonesPS
import com.example.meepmeepsequences.Robot.pwrShotLocal
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import java.util.*

class Path {

    // Declare a MeepMeep instance
    // With a field size of 800 pixels
    val aggroPath = MeepMeep(800) // Set field image
        .setBackground(Background.FIELD_FREIGHT_FRENZY) // Set theme
        .setTheme(ColorSchemeRedDark())
        // Background opacity from 0-1
        .setBackgroundAlpha(1f) // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(54.0, 54.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
        .followTrajectorySequence(
            object : AddTrajectorySequenceCallback {
                override fun buildTrajectorySequence(drive: DriveShim): TrajectorySequence {
                    return drive.trajectorySequenceBuilder(Pose2d(-62.0, 16.8475, 0.0))
                        .lineToSplineHeading(Pose2d(46.0, 16.8475))
                        .splineTo(Vector2d(58.0, 10.0), Math.toRadians(-90.0))
                        .splineTo(Vector2d(58.0, -17.0), Math.toRadians(-90.0))
                        .addFutureTrajectory(object : FutureCallback {
                            override fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder {
                                return builder
                                    .setReversed(true)
                                    .splineTo(Vector2d(58.0, 5.0), Math.toRadians(90.0))
                                    .splineTo(dropZonesPS()[2].vec(), dropZonesPS()[2].heading)
                                    .waitCondition { false }
                                    .setReversed(false)
                                    .splineTo(Vector2d(-5.0, 16.7), Math.toRadians(180.0))
                            }
                        }, Pose2d(-5.0, 16.7, Math.toRadians(180.0)))
                        .waitCondition { false }
//                        .UNSTABLE_addTemporalMarkerOffset(0.4) {
//                            magazine.magMacro()
//                            shooter.state = Shooter.State.CONTINUOUS
//                        }
                        // .addDisplacementMarker { intake.setPower(1.0) }
                        .splineTo(Vector2d(-55.0, 16.7), Math.toRadians(180.0)) // Intake starter rings
                        .setReversed(true)
                        // .prepShooter(0.5, robot)
                        .splineTo(pwrShotLocal(), 0.0)
                        // .addDisplacementMarker { intake.setPower(1.0) }
                        //.lineToSplineHeading(Pose2d(40.0, 3.0, Math.toRadians(0.0)))
                        .splineTo(Vector2d(40.0, 3.0), Math.toRadians(0.0))
                        .turn(Math.toRadians(-180.0))
                        .setReversed(false)
                        .splineTo(Vector2d(58.5275, 40.0), Math.toRadians(90.0))
                        .setReversed(true)
                        // .prepShooter(0.5, robot)
                        .splineTo(Vector2d(20.0, 16.0), Math.toRadians(180.0))
                        .splineTo(Vector2d(-5.8, 17.0), Math.toRadians(180.0))
                        .lineTo(Vector2d(12.0, 17.0))
                        .build()
                }
            }
        )
}
fun TrajectorySequenceBuilder.waitCondition(condition: () -> Boolean): TrajectorySequenceBuilder {
    this.waitSeconds(1.0)
    return this
}

fun TrajectorySequenceBuilder.addFutureTrajectory(callback: FutureCallback, pose: Pose2d) : TrajectorySequenceBuilder {
    return callback.buildFutureSequence(this)
}

fun TrajectorySequenceBuilder.addTrajectorySegment(segment: TrajectorySegment) : TrajectorySequenceBuilder {
    addTrajectory(segment.trajectory)
    return this
}

interface FutureCallback {
    fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder
}

