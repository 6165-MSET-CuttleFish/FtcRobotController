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
    return addTrajectory(segment.trajectory)
}

fun TrajectorySequenceBuilder.liftUp(deposit: Deposit): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Lift Up")
    }
}

fun TrajectorySequenceBuilder.intakeOn(intake: Intake): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Intake On")
    }
}

fun TrajectorySequenceBuilder.intakeOff(intake: Intake): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.0) {
        intake.setPower(0.0)
    }
}

fun TrajectorySequenceBuilder.capstoneReady(capstone: Capstone): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.0, capstone::ready)
}

fun TrajectorySequenceBuilder.dump(platform: Platform): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.0, platform::dump)
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
    fun setPower(power: Double) {
        println("Intake Power $power")
    }
}

class Deposit {
    enum class State {
        LEVEL3,
        LEVEL2,
        IDLE
    }
    fun setState(state: State) {
        println("Lift State $state")
    }
}

class Platform {
    fun dump() {
        print("Dump Freight")
    }
}

class Capstone {
    fun pickUp() {
        println("Capstone retrieved")
    }
    fun ready() {
        println("Capstone picked up")
    }
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