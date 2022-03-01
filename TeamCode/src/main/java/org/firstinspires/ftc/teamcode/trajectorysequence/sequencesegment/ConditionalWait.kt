package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

class ConditionalWait(startPose: Pose2d, markers: List<TrajectoryMarker>, var condition: () -> Boolean, duration: Double = 1.5) : SequenceSegment(
    duration,
    startPose,
    startPose,
    markers
)