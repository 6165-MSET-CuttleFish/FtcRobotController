package org.firstinspires.ftc.teamcode.Components.localizer

import android.content.Context
import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.intel.realsense.librealsense.UsbUtilities
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera
import org.firstinspires.ftc.teamcode.util.rotationRad
import org.firstinspires.ftc.teamcode.util.*

object t265 {
    lateinit var cam: T265Camera

    private val cameraToRobot = Transform2d(Translation2d(0.0, 0.0), Rotation2d(0.0.toRadians()))
    lateinit var update: T265Camera.CameraUpdate

    @JvmStatic
    private var isStarted = false
    val dash = FtcDashboard.getInstance()

    @JvmStatic
    fun init(hardwareMap: HardwareMap) {
        if (!isStarted) {
            try {
                UsbUtilities.grantUsbPermissionIfNeeded(hardwareMap.appContext);
                Log.i("INTEL LOCALIZER", "Camera INITIALIZING")
                cam = T265Camera(cameraToRobot, 0.0, hardwareMap.appContext)
                Log.i("INTEL LOCALIZER", "Camera INITALIZED")
                cam.start()
            } catch (e: Exception) {
                if (e == RuntimeException("T265 camera is already started")) {
                    isStarted = true
                    return
                }
                init(hardwareMap)
                isStarted = false
                Log.e("INTEL LOCALIZER", e.toString())

            }
        }
        update = cam.lastReceivedCameraUpdate

    }

    fun init(context: Context, manager: AnnotatedOpModeManager) {
        if (!isStarted) {
            try {
                UsbUtilities.grantUsbPermissionIfNeeded(context);
                Log.i("INTEL LOCALIZER", "Camera INITIALIZING")
                cam = T265Camera(cameraToRobot, 0.0, context)
                Log.i("INTEL LOCALIZER", "Camera INITALIZED")
                cam.start()
            } catch (e: Exception) {
                if (e == RuntimeException("T265 camera is already started")) {
                    isStarted = true
                } else {
                    init(context, manager)
                    isStarted = false
                    Log.e("INTEL LOCALIZER", e.toString())
                }
            }
        }
        update = cam.lastReceivedCameraUpdate
    }

    var pose: Pose2d
        get() = Pose2d(update.pose.x, update.pose.y, update.pose.rotationRad - 0.0.toRadians()).toInches()
        set(value) {
            cam.setPose(com.arcrobotics.ftclib.geometry.Pose2d(value.x, value.y, Rotation2d(value.heading)).toMeters())
        }

    val speeds: Pose2d
        get() = update.velocity.toRRPose2d()

    val confidence: T265Camera.PoseConfidence
        get() = update.confidence

    fun update() {
        update = cam.lastReceivedCameraUpdate
        Log.i("INTEL LOCALIZER", pose.toString())
        val packet = TelemetryPacket()
        packet.fieldOverlay().fillCircle(pose.x, pose.y, 9.0)
        dash.sendTelemetryPacket(packet)
    }

    fun resetPose() {
        cam.setPose(com.arcrobotics.ftclib.geometry.Pose2d(0.0, 0.0, Rotation2d( 0.0)))
    }
}