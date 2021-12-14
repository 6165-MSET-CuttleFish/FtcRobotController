package org.firstinspires.ftc.teamcode.modules

interface StateBuilder {
    val time: Double
    val isTransitionState: Boolean
    val nextState: StateBuilder?
}