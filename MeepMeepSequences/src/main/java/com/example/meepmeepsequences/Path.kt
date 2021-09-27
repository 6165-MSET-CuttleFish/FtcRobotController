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
                    return drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
                        .forward(20.0)
                        .turn(1.6)
                        .forward(20.0)
                        .turn(1.6)
                        .forward(20.0)
                        .turn(-1.55)
                        .forward(20.0)
                        .turn(-1.55)
                        .waitCondition { false }
//                        .UNSTABLE_addTemporalMarkerOffset(0.4) {
//                            magazine.magMacro()
//                            shooter.state = Shooter.State.CONTINUOUS
//                        }
                        // .addDisplacementMarker { intake.setPower(1.0) }
                        .build()
                }
            }
        )
            .start();
}
