package org.firstinspires.ftc.teamcode.auto.util

import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.TrajectorySegment

fun TrajectorySequenceBuilder.addTrajectorySegment(segment: TrajectorySegment): TrajectorySequenceBuilder {
    return addTrajectory(segment.trajectory)
}

fun TrajectorySequenceBuilder.liftUp(deposit: Deposit): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
        deposit.setState(Deposit.State.LEVEL3)
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