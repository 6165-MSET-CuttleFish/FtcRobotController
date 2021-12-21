package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.localization.Localizer

// TODO : Complete
class PartialLocalizer (val partialCondition: () -> Boolean, vararg allLocalizers: ImprovedLocalizer) : Localizer {
    override var poseEstimate = Pose2d()
        set (value) {
            field = value
            for (localizer in allLocalizers) {
                localizer.poseEstimate = value
            }
        }
    override val poseVelocity = Pose2d()
    private var primaryLocalizer: ImprovedLocalizer = allLocalizers[0]
    private var allLocalizers: Array<ImprovedLocalizer> = allLocalizers as Array<ImprovedLocalizer>

    override fun update() {
        for (localizer in allLocalizers) {
            localizer.update()
        }
        if (partialCondition()) {

        } else {
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, primaryLocalizer.translation)
        }
    }

}