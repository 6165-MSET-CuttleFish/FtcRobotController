package org.firstinspires.ftc.teamcode.Components.localizer

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap

class IntelLocalizer(hardwareMap: HardwareMap, startPose2d: Pose2d) : Localizer {


    /**
     * Current robot pose estimate.
     */
    override var poseEstimate: Pose2d
        get() = t265.pose
        set(value) {
            t265.cam.setPose(com.arcrobotics.ftclib.geometry.Pose2d(value.x, value.y, Rotation2d(value.heading)))
        }

    /**
     * Current robot pose velocity (optional)
     */
    override val poseVelocity: Pose2d?
        get() = TODO("Not yet implemented")

    /**
     * Current robot pose velocity (optional)
     */


    /**
     * Completes a single localization update.
     */
    override fun update() {

        t265.update()
    }


}