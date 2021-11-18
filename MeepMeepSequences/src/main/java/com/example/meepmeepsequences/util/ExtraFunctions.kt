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

fun TrajectorySequenceBuilder.liftUp(lift: Lift): TrajectorySequenceBuilder {
    UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Lift Up")
    }
    return this
}

fun TrajectorySequenceBuilder.intakeOn(intake: Intake): TrajectorySequenceBuilder {
    UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Intake On")
    }
    return this
}

fun TrajectorySequenceBuilder.intakeOff(intake: Intake): TrajectorySequenceBuilder {
    UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Intake Off")
    }
    return this
}

fun TrajectorySequenceBuilder.capstoneReady(capstone: Capstone): TrajectorySequenceBuilder {
    UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Capstone Ready")
    }
    return this
}

fun MeepMeep.configure(): MeepMeep {
    return this
        .setDriveTrainType(DriveTrainType.TANK)
        .setDarkMode(true)
        .setBotDimensions(18.0, 18.0)
        .setConstraints(80.0, 80.0, Math.toRadians(720.0), Math.toRadians(720.0), 15.0)
}

interface FutureCallback {
    fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder
}

class Intake {

}

class Lift {

}

class Capstone {

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