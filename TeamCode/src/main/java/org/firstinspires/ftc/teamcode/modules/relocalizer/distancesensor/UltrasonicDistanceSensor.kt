package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.modules.Module

abstract class UltrasonicDistanceSensor<T> @JvmOverloads constructor(
    hardwareMap: HardwareMap,
    _state: T,
    poseOffset: Pose2d = Pose2d()
) : Module<T>(hardwareMap, _state, poseOffset), DistanceSensor {
    abstract val distance: Double
}