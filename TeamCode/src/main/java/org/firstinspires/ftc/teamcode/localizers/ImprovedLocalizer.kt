package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer

interface ImprovedLocalizer : Localizer {
    val translation: Pose2d
    val weight: Double
}