package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.geometry.Pose2d

object Context {
    @JvmField
    var robotPose = Pose2d()
    @JvmField
    var poseVelocity = Pose2d()
    @JvmField
    var alliance = Alliance.NONE
    @JvmField
    var side = Side.NONE
    val futurePose: Pose2d
        get() = robotPose.plus(poseVelocity)
    @JvmField
    var location = Detector.Location.LEFT

    @JvmField
    var windowSize = 1000
}