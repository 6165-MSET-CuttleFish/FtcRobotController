package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.profile.AccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getAccelerationConstraint
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.waitWhile(condition: () -> Boolean): TrajectorySequenceBuilder {
    return this.waitSeconds(0.5)
}

fun TrajectorySequenceBuilder.waitSeconds(seconds: Double, driveSignal: DriveSignal): TrajectorySequenceBuilder {
    return this.waitSeconds(seconds)
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

fun TrajectorySequenceBuilder.liftUp(deposit: Deposit, level: Deposit.State): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
        deposit.setState(level)
    }
}

fun TrajectorySequenceBuilder.intakeOn(intake: Intake): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
        intake.setPower(1.0)
    }
}

fun TrajectorySequenceBuilder.intakeOff(intake: Intake): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
        intake.setPower(0.0)
    }
}

fun TrajectorySequenceBuilder.capstoneReady(capstone: Capstone): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, capstone::ready)
}

fun TrajectorySequenceBuilder.capstonePickup(capstone: Capstone): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, capstone::pickUp)
}

fun TrajectorySequenceBuilder.dump(deposit: Deposit): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, deposit::dump)
}

fun TrajectorySequenceBuilder.carouselOn(carousel: Carousel): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, carousel::on)
}

fun TrajectorySequenceBuilder.carouselOff(carousel: Carousel): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, carousel::off)
}

fun MeepMeep.configure(): MeepMeep {
    return this
        .setDriveTrainType(DriveTrainType.TANK)
        .setDarkMode(true)
        .setBotDimensions(17.2, 17.192913)
        .setConstraints(60.0, 60.0, Math.toRadians(274.5043079608481), Math.toRadians(274.5043079608481), 15.0)
}

fun TrajectorySequenceBuilder.decreaseGains() = UNSTABLE_addTemporalMarkerOffset(0.0) {
    println("Gains Decreased")
}.setConstraints(
        getVelocityConstraint(
            20.0,
            Math.toRadians(274.0),
            15.0
        ), getAccelerationConstraint(30.0) ?: ProfileAccelerationConstraint(20.0)
    )

fun TrajectorySequenceBuilder.defaultGains() = UNSTABLE_addTemporalMarkerOffset(0.0) {
        println("Gains Default")
    }.setConstraints(
        getVelocityConstraint(
            60.0,
            Math.toRadians(274.0),
            15.0
        ), getAccelerationConstraint(60.0) ?: ProfileAccelerationConstraint(60.0)
    )

interface FutureCallback {
    fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder
}