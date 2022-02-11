package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.deposit.Platform
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.relocalizer.Relocalizer
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.TrajectorySegment

fun TrajectorySequenceBuilder.addTrajectorySegment(segment: TrajectorySegment): TrajectorySequenceBuilder {
    return addTrajectory(segment.trajectory)
}

fun TrajectorySequenceBuilder.relocalize(robot: Robot, offset: Double = 0.0) = UNSTABLE_addDisplacementMarkerOffset(offset) {
    robot.correctPosition()
}

fun TrajectorySequenceBuilder.increaseGains() : TrajectorySequenceBuilder {
    setConstraints(
        Robot.getVelocityConstraint(10.0, Math.toRadians(180.0), DriveConstants.TRACK_WIDTH),
        Robot.getAccelerationConstraint(10.0)
    )
    Robot.isGainScheduled = true
    return this
}

fun TrajectorySequenceBuilder.liftUp(deposit: Deposit, level: Deposit.State): TrajectorySequenceBuilder {
    return UNSTABLE_addDisplacementMarkerOffset(0.0) {
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

fun TrajectorySequenceBuilder.dump(deposit: Deposit): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, deposit::dump)
}

fun TrajectorySequenceBuilder.carouselOn(carousel: Carousel): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, carousel::on)
}

fun TrajectorySequenceBuilder.carouselOff(carousel: Carousel): TrajectorySequenceBuilder {
    return UNSTABLE_addTemporalMarkerOffset(0.0, carousel::off)
}