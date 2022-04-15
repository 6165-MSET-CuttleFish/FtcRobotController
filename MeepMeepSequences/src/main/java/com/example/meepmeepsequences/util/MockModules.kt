package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.geometry.polarAdd
import kotlin.math.cos

class Intake : Module() {
    fun setPower(power: Double) {
        println("Intake Power $power")
    }
}

class Deposit : Module() {
    private val platform = Platform()
    enum class Level {
        LEVEL3,
        LEVEL2,
        IDLE
    }
    fun setState(state: Level) {
        println("Lift State $state")
    }
    fun dump() = platform.dump()
}
class Detector {
    enum class Location {
        LEFT, MIDDLE, RIGHT
    }
}

class Platform : Module() {
    fun dump() {
        println("Dump Freight")
    }
}

class Capstone : Module() {
    fun pickUp() {
        println("Capstone retrieved")
    }
    fun ready() {
        println("Capstone picked up")
    }
}

class Carousel : Module() {
    fun on() {
        println("Carousel On")
    }
    fun off() {
        println("Carousel Off")
    }
}
class Relocalizer : Module() {
    companion object {
        @JvmField
        var frontDistanceSensorXOffset = 8.0
        @JvmField
        var horizontalDistanceSensorYOffset = -8.0
        @JvmField
        var frontDistanceSensorYOffset = 8.0
        @JvmField
        var horizontalDistanceSensorXOffset = -8.0
    }
    var poseEstimate = Pose2d()
        private set
    private val pitchOffset: Double = pitch
    private val tiltOffset: Double = tilt

    enum class State {
        RELOCALIZING, IDLE;

        val timeOut = 0.0
    }

    fun update() {
        val frontDist = 8.0 * cos(tilt)
        val horizontalDist = 2.0 * cos(tilt)
        if (Context.side == Side.CAROUSEL) {
//            frontDist = 0.0
//            horizontalDist = 0.0
        } else {
//            frontDist =
//                (if (alliance == Alliance.BLUE) frontRightDistance.distance else frontLeftDistance.distance) * cos(
//                    tilt
//                )
//            horizontalDist =
//                (if (alliance == Alliance.BLUE) leftDistance.distance else rightDistance.distance) * cos(
//                    pitch
//                )
            val frontWallX = 70.5
            val sideWallY = if (alliance == Alliance.BLUE) 70.5 else -70.5
            val heading = Context.robotPose.heading
            val x = frontWallX - frontDist * cos(heading)
            print("X $x ,")
            val y = sideWallY - horizontalDist * cos(heading)
            println("Y $y")
            val xPoseEstimate =
                Pose2d(x, Context.robotPose.y, heading)
                    .polarAdd(-frontDistanceSensorXOffset)
                    .polarAdd(-frontDistanceSensorYOffset, Math.toRadians(90.0))
            val yPoseEstimate =
                Pose2d(Context.robotPose.x, y, heading)
                    .polarAdd(-horizontalDistanceSensorXOffset)
                    .polarAdd(-horizontalDistanceSensorYOffset, Math.toRadians(90.0))
            poseEstimate = Pose2d(xPoseEstimate.x, yPoseEstimate.y, heading)
        }
    }

    private val pitch: Double
        get() = Math.toRadians(30.0)
    private val tilt: Double
        get() = Math.toRadians(30.0)
}

abstract class Module {
    fun isDoingWork(): Boolean {
        return true
    }
    fun isTransitioningState(): Boolean {
        return true
    }
}
