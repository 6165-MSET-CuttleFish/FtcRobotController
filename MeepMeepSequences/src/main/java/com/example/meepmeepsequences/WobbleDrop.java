package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class WobbleDrop{
    public static void main(String[] args){
        MeepMeep mm = new MeepMeep(1000)
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(220), Math.toRadians(220), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-62, -50.7, 0))
                        //if(rings == 0)
                            .lineToLinearHeading(new Pose2d(-5.4725, -55.4, Math.toRadians(-180)))
                        //if(rings == 1)
                            .lineToLinearHeading(new Pose2d(23, -35.4725, Math.toRadians(-180)))
                        //if(rings == 4)
                            .lineToLinearHeading(new Pose2d(45.5275, -57, Math.toRadians(-180)))
                        .build());

    }
}