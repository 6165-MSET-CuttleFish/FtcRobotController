package org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

class ConditionalWait(startPose: Pose2d, markers: List<TrajectoryMarker>, var condition: () -> Boolean, var driveSignal: (Double) -> DriveSignal = { DriveSignal() }) : SequenceSegment(
    { 1.5 },
    startPose,
    startPose,
    markers
)