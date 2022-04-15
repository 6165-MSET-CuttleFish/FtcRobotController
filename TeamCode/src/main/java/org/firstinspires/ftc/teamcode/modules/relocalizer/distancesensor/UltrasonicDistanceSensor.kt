package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.outoftheboxrobotics.neutrinoi2c.MB1242.AsyncMB1242

class UltrasonicDistanceSensor @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    name: String,
    _state: SensorType,
    poseOffset: Pose2d = Pose2d(),
) : Module<UltrasonicDistanceSensor.SensorType>(hardwareMap, _state, poseOffset), DistanceSensor {
    enum class SensorType {
        LongRange,
        ShortRange,
    }
    private val distanceSensor: DistanceSensor =
        if (_state == SensorType.LongRange) MB1242(hardwareMap, name) else MB1643(hardwareMap, name)

    override fun internalInit() {
        if (distanceSensor is MB1242) distanceSensor.setRunDelayMs(100)
    }

    override fun internalUpdate() {
    }

    override fun isDoingInternalWork() = false

    override fun isTransitioningState() = false

    override fun getManufacturer(): HardwareDevice.Manufacturer = distanceSensor.manufacturer

    override fun getDeviceName(): String = distanceSensor.deviceName

    override fun getConnectionInfo(): String = distanceSensor.connectionInfo

    override fun getVersion(): Int = distanceSensor.version

    override fun resetDeviceConfigurationForOpMode() = distanceSensor.resetDeviceConfigurationForOpMode()

    override fun close() = distanceSensor.close()

    override fun getDistance(unit: DistanceUnit?): Double = distanceSensor.getDistance(unit)
}