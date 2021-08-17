package org.firstinspires.ftc.teamcode.localizers

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.spartronics4915.lib.T265Camera
import org.firstinspires.ftc.teamcode.util.toFTCLibPose2d
import org.firstinspires.ftc.teamcode.util.toRRPose2d

import java.io.File

class T265(hardwareMap: HardwareMap) {
    private val odo: StandardTrackingWheelLocalizer
    var veloTimer = ElapsedTime()

    // Constants
    private val ODOMETRY_COVARIANCE = 0.15
    private val INCH_TO_METER = 0.0254

    @SuppressLint("SdCardPath")
    private val mapPath = "/data/user/0/com.qualcomm.ftcrobotcontroller/cache/map.bin"
    var isEmpty = false
    private var exportingMap = true
    fun exportMap() {
        if (exportingMap) {
            exportingMap = false
            t265Cam!!.exportRelocalizationMap(mapPath)
        }
    }

    fun setCameraPose(x: Double, y: Double, theta: Double) {
        t265Cam?.setPose(
            com.arcrobotics.ftclib.geometry.Pose2d(
                -x * INCH_TO_METER,
                -y * INCH_TO_METER,
                Rotation2d(-theta)
            )
        )
        odo.poseEstimate = Pose2d(x, y, theta)
    }

    fun updateCamPose() {
        odo.update()
        val state = t265Cam!!.lastReceivedCameraUpdate
        val translation = Translation2d(
            -state.pose.translation.x / INCH_TO_METER,
            -state.pose.translation.y / INCH_TO_METER
        )
        val rotation = state.pose.rotation
        //poseVelo = new Pose2d(temp.getTranslation().minus(curr.getTranslation()).div(veloTimer.seconds()), new Rotation2d((temp.getHeading() - curr.getHeading()) / veloTimer.seconds()));
        curr = com.arcrobotics.ftclib.geometry.Pose2d(translation, rotation)
        val currVelo = odo.poseVelocity
        Companion.poseVelo = odo.poseVelocity?.toFTCLibPose2d() ?: com.arcrobotics.ftclib.geometry.Pose2d()
        if (currVelo != null) {
            try {
                t265Cam!!.sendOdometry(currVelo.x * INCH_TO_METER, currVelo.y * INCH_TO_METER)
            } catch (ignored: Exception) {
            }
        }
        odo.poseEstimate = Pose2d(odo.poseEstimate.x, odo.poseEstimate.y, curr.heading)
        veloTimer.reset()
    }

    val pose: Pose2d
        get() = curr.toRRPose2d()
    val poseVelo: com.arcrobotics.ftclib.geometry.Pose2d
        get() = Companion.poseVelo

    companion object {
        // Electronics
        private var t265Cam: T265Camera? = null
        var curr = com.arcrobotics.ftclib.geometry.Pose2d()
        var poseVelo = com.arcrobotics.ftclib.geometry.Pose2d()
        @kotlin.jvm.JvmStatic
        fun startCam() {
            if (!t265Cam!!.isStarted) t265Cam!!.start()
        }

        @kotlin.jvm.JvmStatic
        fun stopCam() {
            t265Cam!!.stop()
        }
    }

    init {
        val file = File(mapPath)
        if (!file.exists() || file.length() == 0L) {
            isEmpty = true
        }
        if (t265Cam == null) {
            if (!isEmpty) {
                t265Cam = T265Camera(
                    Transform2d(
                        Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), Rotation2d(
                            Math.toRadians(0.0)
                        )
                    ), ODOMETRY_COVARIANCE, mapPath, hardwareMap.appContext
                )
            } else {
                t265Cam = T265Camera(
                    Transform2d(
                        Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), Rotation2d(
                            Math.toRadians(0.0)
                        )
                    ), ODOMETRY_COVARIANCE, hardwareMap.appContext
                )
            }
        }
        odo = StandardTrackingWheelLocalizer(hardwareMap)
    }
}