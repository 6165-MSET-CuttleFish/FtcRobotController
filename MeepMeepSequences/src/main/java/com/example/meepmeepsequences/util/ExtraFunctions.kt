package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.waitCondition(condition: () -> Boolean): TrajectorySequenceBuilder {
    return this.waitSeconds(0.5)
}

/**
 * create a trajectory that will be fulfilled in the future
 */
fun TrajectorySequenceBuilder.addFutureTrajectory(callback: FutureCallback, pose: Pose2d): TrajectorySequenceBuilder {
    return callback.buildFutureSequence(this)
}

fun TrajectorySequenceBuilder.addTrajectorySegment(segment: TrajectorySegment): TrajectorySequenceBuilder {
    addTrajectory(segment.trajectory)
    return this
}

fun MeepMeep.configure(): MeepMeep {
    return this
        .setDriveTrainType(DriveTrainType.TANK)
        .setConstraints(80.0, 80.0, Math.toRadians(720.0), Math.toRadians(720.0), 15.0)
}

interface FutureCallback {
    fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder
}

fun Double.flip(negative: Boolean): Double {
    if (negative)
        return -this
    return this
}

fun Pose2d.flip(negative: Boolean): Pose2d {
    if (negative)
        return Pose2d(this.x, -this.y, -this.heading)
    return this
}

fun Double.random(): Double {
    var multiplier = 1
    val rand = Math.random()
    if (rand < 0.5) multiplier = -1
    return Math.random() * multiplier * this
}

fun Vector2d.flip(negative: Boolean): Vector2d {
    if (negative)
        return Vector2d(this.x, -this.y)
    return this
}