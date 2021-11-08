package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.waitCondition(condition: () -> Boolean): TrajectorySequenceBuilder {
    return this.waitSeconds(1.0)
}

/**
 * create a trajectory that will be fulfilled in the future
 */
fun TrajectorySequenceBuilder.addFutureTrajectory(callback: FutureCallback, pose: Pose2d) : TrajectorySequenceBuilder {
    return callback.buildFutureSequence(this)
}

fun TrajectorySequenceBuilder.addTrajectorySegment(segment: TrajectorySegment) : TrajectorySequenceBuilder {
    addTrajectory(segment.trajectory)
    return this
}

interface FutureCallback {
    fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder
}