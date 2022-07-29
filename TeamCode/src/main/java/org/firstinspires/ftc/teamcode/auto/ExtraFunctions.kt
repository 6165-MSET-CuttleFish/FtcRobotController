package org.firstinspires.ftc.teamcode.auto

import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequenceBuilder

fun TrajectorySequenceBuilder<*>.foo(module: Module<*>): TrajectorySequenceBuilder<*> {
    return performAction {
        // do something with the module
    }
}