package org.firstinspires.ftc.teamcode.localizers

import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.intel.realsense.librealsense.UsbUtilities
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import com.spartronics4915.lib.T265Camera
import org.firstinspires.ftc.teamcode.util.*

/**
 * Static accessibility class which allows for easy
 * creation and access of the T265 camera, wrapping
 * the FTC265 library to allow more straightforward
 * usage.
 */
@Suppress("UNUSED")
object Easy265 {

    private const val TAG = "Easy265"
    private val ODOMETRY_COVARIANCE = 1.0
    private val INCH_TO_METER = 0.0254

    /**
     * The global T265 camera, throws UninitializedPropertyAccessException
     * if the camera instance hasn't been created, call init() to do so
     */
    @JvmStatic lateinit var camera: T265Camera
        private set

    @JvmStatic val isStarted get() = ::camera.isInitialized && camera.isStarted

    /**
     * The camera data returned from the last update.
     * Returns null if update() hasn't been called or the camera hasn't been started
     */
    @JvmStatic var lastCameraUpdate: T265Camera.CameraUpdate? = null

    /**
     * The pose reported from the last camera update, in inches.
     * Returns null if update() hasn't been called or the camera hasn't been started
     * The pose is only sent to the camera if it isn't null and the camera has been started.
     */
    @JvmStatic var lastPose
        //set and get are in inches
        set(value) {
            if(value != null && isStarted) {
                Async.start {
                    while (lastCameraUpdate?.confidence != T265Camera.PoseConfidence.High) {
                        camera.setPose(value.toFTCLibPose2d().toMeters())
                    }
                    camera.setPose(value.toFTCLibPose2d().toMeters())
                }

            }
        }
        get() = lastCameraUpdate?.pose?.toRRPose2d()?.toInches()

    /**
     * The velocity reported from the last camera update.
     * Returns null if update() hasn't been called or the camera hasn't been started
     */
    @JvmStatic val lastVelocity get() = lastCameraUpdate?.velocity?.toRRPoseVelo()?.toInches()

    /**
     * Initializes and starts the camera. If it has already been started,
     * it updates the data. Recursively tries to open the camera the specified
     * amount of attempts.
     *
     * @param hardwareMap current HardwareMap (tp get the appContext)
     * @param cameraToRobot offset of the center of the robot from the center of the camera
     * @param attempts amount of times this method will be called recursively if the initialization fails
     */
    @JvmStatic @JvmOverloads fun init(
        hardwareMap: HardwareMap,
        cameraToRobot: Transform2d = Transform2d(Translation2d(0.0, 0.0), Rotation2d(0.0)),
        attempts: Int = 6
    ) {
        try {
            if(!::camera.isInitialized) {
                UsbUtilities.grantUsbPermissionIfNeeded(hardwareMap.appContext)
                camera = T265Camera(
                    Transform2d(
                        Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), Rotation2d(
                            Math.toRadians(0.0)
                        )
                    ), ODOMETRY_COVARIANCE, hardwareMap.appContext
                )
                UsbUtilities.grantUsbPermissionIfNeeded(hardwareMap.appContext)
            }

            if(camera.isStarted) {
                update()
                return
            }

            camera.start()
            update()
        } catch(e: Exception) {
            if(attempts <= 0) {
                throw RuntimeException("Unable to start T265Camera after various attempts", e)
            } else {
                RobotLog.w(TAG, "Unable to start T265Camera, retrying...", e)
                init(hardwareMap, cameraToRobot, attempts - 1)
            }
        }
    }

    /**
     * Initializes and start the camera. If it has already been started,
     * it simply updates the data. Infinitely tries to recursively open the
     * camera. Not recommended.
     *
     * @param hardwareMap current HardwareMap (tp get the appContext)
     * @param cameraToRobot offset of the center of the robot from the center of the camera
     */
    @JvmStatic @JvmOverloads fun initWithoutStop(
        hardwareMap: HardwareMap,
        cameraToRobot: Transform2d = Transform2d(
            Translation2d(-8 * INCH_TO_METER, 0 * INCH_TO_METER), Rotation2d(
                Math.toRadians(0.0)
            )
        )
    ) = init(hardwareMap, cameraToRobot, Int.MAX_VALUE)

    /**
     * Pulls the last data from the camera and stores it the
     * lastCameraUpdate variable, therefore updating the data
     * returned by the lastPose or lastVelocity properties.
     *
     * If the camera hasn't been started (or instantiated) no update happens.
     */
    @JvmStatic fun update() {
        if(isStarted) {
            lastCameraUpdate = camera.lastReceivedCameraUpdate
        }
    }

}