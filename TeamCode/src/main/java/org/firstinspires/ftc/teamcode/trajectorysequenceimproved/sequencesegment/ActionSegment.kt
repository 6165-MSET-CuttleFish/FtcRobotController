package org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

class ActionSegment(startPose: Pose2d, markers: List<TrajectoryMarker>, var callback: MarkerCallback) : SequenceSegment(
    { 0.0 },
    startPose,
    startPose,
    markers
)