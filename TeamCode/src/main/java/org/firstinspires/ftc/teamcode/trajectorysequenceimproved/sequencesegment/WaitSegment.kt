package org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker

class WaitSegment(pose: Pose2d, seconds: Double, markers: List<TrajectoryMarker>, val driveSignal: DriveSignal = DriveSignal()) :
    SequenceSegment({ seconds }, pose, pose, markers)