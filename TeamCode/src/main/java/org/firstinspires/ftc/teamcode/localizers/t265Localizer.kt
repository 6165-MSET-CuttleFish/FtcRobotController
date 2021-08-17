package org.firstinspires.ftc.teamcode.localizers

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.toRRPose2d

class t265Localizer @SuppressLint("SdCardPath") constructor(hardwareMap: HardwareMap) : Localizer {
    val cam: T265 = T265(hardwareMap)
    override var poseEstimate: Pose2d
        get() = cam.pose
        set(pose2d) {
            cam.setCameraPose(pose2d.x, pose2d.y, pose2d.heading)
        }

    override fun update() {
        cam.updateCamPose()
    }

    override val poseVelocity: Pose2d?
        get() = cam.poseVelo.toRRPose2d()

    init {
        try {
            Thread.sleep(1000)
        } catch (e: InterruptedException) {
            e.printStackTrace()
        }
        T265.startCam()
        update()
    }
}