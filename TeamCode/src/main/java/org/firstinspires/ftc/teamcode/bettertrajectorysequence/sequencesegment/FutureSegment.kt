package org.firstinspires.ftc.teamcode.bettertrajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.bettertrajectorysequence.TrajectorySequenceBuilder

class FutureSegment(
    startPose2d: Pose2d,
    endPose2d: Pose2d,
    var trajectory: TrajectorySequence? = null,
) : SequenceSegment(
    {
        trajectory?.duration() ?: 0.0
    },
    startPose2d,
    endPose2d,
    emptyList(),
)