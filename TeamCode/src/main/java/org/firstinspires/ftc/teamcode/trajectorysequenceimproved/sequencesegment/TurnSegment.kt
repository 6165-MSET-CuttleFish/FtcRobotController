package org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker
import com.acmerobotics.roadrunner.util.Angle.norm

class TurnSegment(
    startPose: Pose2d,
    val totalRotation: Double,
    val motionProfile: MotionProfile,
    markers: List<TrajectoryMarker>
) : SequenceSegment(
    { motionProfile.duration() },
    startPose,
    Pose2d(
        startPose.x, startPose.y,
        norm(startPose.heading + totalRotation)
    ),
    markers
)