package org.firstinspires.ftc.teamcode.auto

import org.firstinspires.ftc.teamcode.drive.DriveConstants.*
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.deposit.Platform
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.TrajectorySegment

fun TrajectorySequenceBuilder.addTrajectorySegment(segment: TrajectorySegment): TrajectorySequenceBuilder {
    return addTrajectory(segment.trajectory)
}

fun TrajectorySequenceBuilder.liftUp(deposit: Deposit, level: Deposit.State): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.01) {
        Platform.isLoaded = true
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

fun TrajectorySequenceBuilder.increaseGains() : TrajectorySequenceBuilder = UNSTABLE_addTemporalMarkerOffset(0.0) {
    kV *= 5
}.setConstraints(
        Robot.getVelocityConstraint(
            40.0,
            MAX_ANG_VEL,
            TRACK_WIDTH
        ), Robot.getAccelerationConstraint(40.0)
    )

fun TrajectorySequenceBuilder.defaultGains() : TrajectorySequenceBuilder = UNSTABLE_addTemporalMarkerOffset(0.0) {
    kV /= 5
}.resetConstraints()

fun TrajectorySequenceBuilder.capstoneReady(capstone: Capstone): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {}
}

fun TrajectorySequenceBuilder.capstonePickup(capstone: Capstone): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {}
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