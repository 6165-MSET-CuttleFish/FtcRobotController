package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;

public class Javapath {
    public MeepMeep path = new MeepMeep(800)
    .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY) // Set theme
        .setTheme(new ColorSchemeRedDark())
            .setDriveTrainType(DriveTrainType.TANK)
    // Background opacity from 0-1
        .setBackgroundAlpha(1f) // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(54.0, 54.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d())
//                        .UNSTABLE_addTemporalMarkerOffset(0.4) {
//                            magazine.magMacro()
//                            shooter.state = Shooter.State.CONTINUOUS
//                        }
            // .addDisplacementMarker { intake.setPower(1.0) }
                        .splineTo(new Vector2d(-55.0, 16.7), Math.toRadians(180.0)) // Intake starter rings
                    .setReversed(true)
                    // .prepShooter(0.5, robot)
                    // .addDisplacementMarker { intake.setPower(1.0) }
                    //.lineToSplineHeading(Pose2d(40.0, 3.0, Math.toRadians(0.0)))

                    .build()

        );

}
