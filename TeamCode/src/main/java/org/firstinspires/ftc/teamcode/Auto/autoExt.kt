package org.firstinspires.ftc.teamcode.Auto

import org.firstinspires.ftc.teamcode.Components.Gunner
import org.firstinspires.ftc.teamcode.Components.Magazine
import org.firstinspires.ftc.teamcode.Components.Robot
import org.firstinspires.ftc.teamcode.Components.Shooter
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.shoot(gunner: Gunner): TrajectorySequenceBuilder {
    this.UNSTABLE_addTemporalMarkerOffset(0.0) { gunner.shoot() }
    return this
}

fun TrajectorySequenceBuilder.magMacro(offset: Double, magazine: Magazine): TrajectorySequenceBuilder {
    this.UNSTABLE_addTemporalMarkerOffset(offset) { magazine.magMacro() }
    return this
}

fun TrajectorySequenceBuilder.prepShooter(offset: Double, robot: Robot): TrajectorySequenceBuilder {
    this.UNSTABLE_addTemporalMarkerOffset(offset) {
        robot.intake.setPower(0.0)
        robot.shooter.magazine.magMacro()
        robot.shooter.state = Shooter.State.POWERSHOTS
    }
    return this
}