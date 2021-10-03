package org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

abstract class SequenceSegment protected constructor(
    val duration: () -> Double,
    var startPose: Pose2d, var endPose: Pose2d,
    val markers: List<TrajectoryMarker>
)