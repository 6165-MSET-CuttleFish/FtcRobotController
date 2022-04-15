package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.geometry.*
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.DriveTrainType
import com.noahbres.meepmeep.roadrunner.SampleTankDrive.Companion.getVelocityConstraint
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.waitWhile(condition: () -> Boolean): TrajectorySequenceBuilder {
    return this.waitSeconds(0.3)
}

fun TrajectorySequenceBuilder.splineToVectorOffset(endTangentVector: Vector2d, offset: Pose2d, endTangent: Double) : TrajectorySequenceBuilder {
    val vector2d = endTangentVector.polarAdd(-offset.x, endTangent).polarAdd(-offset.y, endTangent + Math.PI / 2)
    return this.splineTo(vector2d, endTangent)
}

fun TrajectorySequenceBuilder.waitSeconds(seconds: Double, driveSignal: DriveSignal): TrajectorySequenceBuilder {
    return this.waitSeconds(seconds)
}

fun TrajectorySequenceBuilder.splineTo(endPosition: Vector2d, endTangent: Vector2d, offset: Pose2d = Pose2d()) = this.splineTo(endPosition + offset.vec(), endPosition.angleTo(endTangent) + offset.heading)

fun TrajectorySequenceBuilder.splineToCircle(circle: Circle, line: Line, reference: Vector2d) : TrajectorySequenceBuilder {
    val endPose = Coordinate.lineCircleIntersection(circle, Coordinate.toPoint(line.start), Coordinate.toPoint(line.end)).minByOrNull { it.distTo(reference) }
    return this.splineTo(endPose ?: reference, endPose?.angleTo(circle.center) ?: reference.angleTo(circle.center))
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

fun TrajectorySequenceBuilder.relocalize(robot: DriveShim): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
//        relocalizer.update()
//        println("${relocalizer.poseEstimate.x}, ${relocalizer.poseEstimate.y}, ${Math.toDegrees(relocalizer.poseEstimate.heading)}")
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

fun DefaultBotBuilder.configure(): DefaultBotBuilder {
    return this
        .setDriveTrainType(DriveTrainType.TANK)
        .setDimensions(16.877953, 16.1417)
        .setConstraints(50.0, 150.0, Math.toRadians(774.5043079608481), Math.toRadians(774.5043079608481), 14.42126)
}

fun MeepMeep.addMultiPath(botEntityBuilder: (Boolean, MeepMeep) -> RoadRunnerBotEntity): MeepMeep {
    return this
        .addEntity(botEntityBuilder(true, this))
        .addEntity(botEntityBuilder(false, this))
}

fun TrajectorySequenceBuilder.increaseGains(amount: Double = 40.0) = UNSTABLE_addDisplacementMarkerOffset(0.0) {
    println("Gains Increased")
}.setVelConstraint(
        getVelocityConstraint(
            amount,
            Math.toRadians(274.0),
            15.0
        )
    )

fun TrajectorySequenceBuilder.defaultGains() = UNSTABLE_addDisplacementMarkerOffset(0.0) {
        println("Gains Default")
    }.resetConstraints()

interface FutureCallback {
    fun buildFutureSequence(builder: TrajectorySequenceBuilder): TrajectorySequenceBuilder
}