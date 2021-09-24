package org.firstinspires.ftc.teamcode.Auto

import org.firstinspires.ftc.teamcode.Components.Gunner
import org.firstinspires.ftc.teamcode.Components.Magazine
import org.firstinspires.ftc.teamcode.Components.Robot
import org.firstinspires.ftc.teamcode.Components.Shooter
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.shoot(gunner: Gunner): TrajectorySequenceBuilder {
    return this.addDisplacementMarker { gunner.shoot() }
}

fun TrajectorySequenceBuilder.magMacro(offset: Double, magazine: Magazine): TrajectorySequenceBuilder {
    return this.UNSTABLE_addTemporalMarkerOffset(offset) { magazine.magMacro() }
}

fun TrajectorySequenceBuilder.prepShooter(offset: Double, robot: Robot, state: Shooter.State): TrajectorySequenceBuilder {
    return this.UNSTABLE_addTemporalMarkerOffset(offset) {
        robot.intake.setPower(0.0)
        robot.shooter.magazine.magMacro()
        robot.shooter.state = state
    }
}

fun TrajectorySequenceBuilder.waitActionsCompleted(robot: Robot) : TrajectorySequenceBuilder {
    return this.waitCondition { !robot.isHazardous }
}