package org.firstinspires.ftc.teamcode.localizers.t265

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.localizers.ImprovedLocalizer
import org.firstinspires.ftc.teamcode.util.field.Context.telemetry

/**
 * A RoadRunner localizer using the Easy265 wrapper
 * to perform localization with a T265 camera.
 */
@Suppress("UNUSED")
class T265Localizer @JvmOverloads constructor(override val weight: Double = 1.0) : ImprovedLocalizer {
    /**
     * Updates the T265 camera and returns the last pose
     * if isn't null from the Easy265 wrapper static class.
     * If it's null, it returns a (0, 0, 0) pose.
     */
    override var poseEstimate: Pose2d
        get() {
            Easy265.update()

            return if (Easy265.lastPose != null) {
                Easy265.lastPose!!
            } else Pose2d(0.0,0.0,0.0)
        }
        set(value) {
            Easy265.lastPose = value
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

    override val translation: Pose2d
        get() {
            Easy265.update()
            return Easy265.lastTranslation
        }

    /**
     * Updates the T265 camera data, held
     * in the Easy265 wrapper static class.
     */
    override fun update() {
        Easy265.update()
        telemetry?.addData("Confidence", Easy265.lastCameraUpdate?.confidence)
    }

}