package org.firstinspires.ftc.teamcode.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer

/**
 * A RoadRunner localizer using the Easy265 wrapper
 * to perform localization with a T265 camera.
 */
@Suppress("UNUSED")
class T265Localizer : Localizer {
    var offset = Pose2d()
    /**
     * Updates the T265 camera and returns the last pose
     * if isn't null from the Easy265 wrapper static class.
     * If it's null, it returns a (0, 0, 0) pose.
     */
    override var poseEstimate: Pose2d
        get() {
            Easy265.update()

            return if (Easy265.lastPose != null) {
                Easy265.lastPose!!.plus(offset)
            } else Pose2d(0.0,0.0,0.0).plus(offset)
        }
        set(value) {
            offset = value.minus(poseEstimate.minus(offset));//sets the offset as negative
        }

    /**
     * Updates the T265 camera and returns the last velocity,
     * from the Easy265 wrapper static class.
     */
    override val poseVelocity: Pose2d?
        get() {
            Easy265.update()
            return Easy265.lastVelocity
        }

    /**
     * Updates the T265 camera data, held
     * in the Easy265 wrapper static class.
     */
    override fun update() = Easy265.update()

}