package org.firstinspires.ftc.teamcode.auto

import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.Robot
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.modules.intake.Intake
import org.firstinspires.ftc.teamcode.modules.relocalizer.Relocalizer
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.util.field.Alliance
import org.firstinspires.ftc.teamcode.util.field.Context

fun TrajectorySequenceBuilder<*>.relocalize(robot: Robot<*>, offset: Double = 0.0) = performAction {
    robot.relocalizer.updatePoseEstimate(
        null,
        if (Context.alliance == Alliance.BLUE) Relocalizer.Sensor.LEFT else Relocalizer.Sensor.RIGHT,
    )
    robot.correctPosition()
}

fun TrajectorySequenceBuilder<*>.increaseGains(velo: Double) : TrajectorySequenceBuilder<*> {
    setVelConstraint(Robot.getVelocityConstraint(velo, Math.toRadians(180.0), DriveConstants.TRACK_WIDTH))
    return this
}

fun TrajectorySequenceBuilder<*>.defaultGains() : TrajectorySequenceBuilder<*> {
    resetConstraints()
    return this
}

fun TrajectorySequenceBuilder<*>.liftLevel(deposit: Deposit, level: Deposit.Level): TrajectorySequenceBuilder<*> {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
        deposit.setLevel(level)
    }
}

fun TrajectorySequenceBuilder<*>.intakeOn(intake: Intake): TrajectorySequenceBuilder<*> {
    return UNSTABLE_addTemporalMarkerOffset(0.0) {
        intake.setPower(1.0)
    }
}

fun TrajectorySequenceBuilder<*>.intakeOff(intake: Intake): TrajectorySequenceBuilder<*> {
    return performAction {
        intake.setPower(0.0)
    }
}

fun TrajectorySequenceBuilder<*>.softDump(deposit: Deposit): TrajectorySequenceBuilder<*> {
    return performAction(deposit::softDump)
}

fun TrajectorySequenceBuilder<*>.hardDump(deposit: Deposit): TrajectorySequenceBuilder<*> {
    return performAction(deposit::dump)
}

fun TrajectorySequenceBuilder<*>.carouselOn(carousel: Carousel): TrajectorySequenceBuilder<*> {
    return UNSTABLE_addTemporalMarkerOffset(0.0, carousel::on)
}

fun TrajectorySequenceBuilder<*>.carouselOff(carousel: Carousel): TrajectorySequenceBuilder<*> {
    return UNSTABLE_addTemporalMarkerOffset(0.0, carousel::off)
}