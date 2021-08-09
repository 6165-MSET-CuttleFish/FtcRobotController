package org.firstinspires.ftc.teamcode.Auto

import org.firstinspires.ftc.teamcode.Components.Gunner
import org.firstinspires.ftc.teamcode.Components.Magazine
import org.firstinspires.ftc.teamcode.Components.Robot
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder.shoot(gunner: Gunner): TrajectorySequenceBuilder {
    this.UNSTABLE_addTemporalMarkerOffset(0.0) { gunner.shoot() }
    return this
}

fun TrajectorySequenceBuilder.magMacro(offset: Double, magazine: Magazine): TrajectorySequenceBuilder {
    this.UNSTABLE_addTemporalMarkerOffset(offset) { magazine.magMacro() }
    return this
}