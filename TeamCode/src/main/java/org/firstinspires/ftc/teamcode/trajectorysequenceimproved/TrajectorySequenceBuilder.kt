package org.firstinspires.ftc.teamcode.trajectorysequenceimproved

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.PathContinuityViolationException
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator.generateSimpleMotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.*
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.Angle.norm
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.angleTo
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.sequencesegment.*
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.flip
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.Circle
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.Coordinate
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.Line
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import kotlin.collections.ArrayList
import kotlin.math.abs
import kotlin.math.min

class TrajectorySequenceBuilder<T>(
    startPose: Pose2d,
    startTangent: Double?,
    private val baseVelConstraint: TrajectoryVelocityConstraint,
    private val baseAccelConstraint: TrajectoryAccelerationConstraint,
    baseTurnConstraintMaxAngVel: Double,
    baseTurnConstraintMaxAngAccel: Double
) {
    private val resolution = 0.25
    private var currentVelConstraint: TrajectoryVelocityConstraint
    private var currentAccelConstraint: TrajectoryAccelerationConstraint
    private val baseTurnConstraintMaxAngVel: Double
    private val baseTurnConstraintMaxAngAccel: Double
    private var currentTurnConstraintMaxAngVel: Double
    private var currentTurnConstraintMaxAngAccel: Double
    private val sequenceSegments: MutableList<SequenceSegment?>
    private val temporalMarkers: MutableList<TemporalMarker>
    private val displacementMarkers: MutableList<DisplacementMarker>
    private val spatialMarkers: MutableList<SpatialMarker>
    private var lastPose: Pose2d
    private var tangentOffset: Double
    private var setAbsoluteTangent: Boolean
    private var absoluteTangent: Double
    private var currentTrajectoryBuilder: TrajectoryBuilder?
    private var currentDuration: Double
    private var currentDisplacement: Double
    private var lastDurationTraj: Double
    private var lastDisplacementTraj: Double
    private var isFlipped = false
    private var state: T? = null

    constructor(
        startPose: Pose2d,
        baseVelConstraint: TrajectoryVelocityConstraint,
        baseAccelConstraint: TrajectoryAccelerationConstraint,
        baseTurnConstraintMaxAngVel: Double,
        baseTurnConstraintMaxAngAccel: Double
    ) : this(
        startPose, null,
        baseVelConstraint, baseAccelConstraint,
        baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
    )

    fun lineTo(endPosition: Vector2d): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineTo(
                    endPosition.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun lineTo(
        endPosition: Vector2d,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineTo(
                    endPosition.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun lineToConstantHeading(endPosition: Vector2d): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToConstantHeading(
                    endPosition.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun lineToConstantHeading(
        endPosition: Vector2d,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToConstantHeading(
                    endPosition.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun lineToLinearHeading(endPose: Pose2d): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToLinearHeading(
                    endPose.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun lineToLinearHeading(
        endPose: Pose2d,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToLinearHeading(
                    endPose.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun lineToSplineHeading(endPose: Pose2d) = addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToSplineHeading(
                    endPose.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })

    fun lineToSplineHeading(
        endPose: Pose2d,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ) = addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.lineToSplineHeading(
                    endPose.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })

    fun strafeTo(endPosition: Vector2d) = addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeTo(
                    endPosition.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })

    fun strafeTo(
        endPosition: Vector2d,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeTo(
                    endPosition.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun forward(distance: Double) = addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.forward(
                    distance,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        })

    fun forward(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ) = addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.forward(
                    distance,
                    velConstraint,
                    accelConstraint
                )
            }
        })

    fun back(distance: Double) = addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.back(
                    distance,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        })

    fun back(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.back(
                    distance,
                    velConstraint,
                    accelConstraint
                )
            }
        })
    }

    fun strafeLeft(distance: Double): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeLeft(
                    distance,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        })
    }

    fun strafeLeft(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeLeft(
                    distance,
                    velConstraint,
                    accelConstraint
                )
            }
        })
    }

    fun strafeRight(distance: Double): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeRight(
                    distance,
                    currentVelConstraint,
                    currentAccelConstraint
                )
            }
        })
    }

    fun strafeRight(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.strafeRight(
                    distance,
                    velConstraint,
                    accelConstraint
                )
            }
        })
    }

    fun splineTo(endPosition: Vector2d, endHeading: Double): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineTo(
                    endPosition.flip(isFlipped), endHeading.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun splineToVectorOffset(endTangentVector: Vector2d, offset: Pose2d, endTangent: Double) : TrajectorySequenceBuilder<T> {
        val vector2d = endTangentVector.polarAdd(-offset.x, endTangent).polarAdd(-offset.y, endTangent + Math.PI / 2)
        return this.splineTo(vector2d, endTangent)
    }

    /**
     * @param reference closest vector to the wanted location in the event that there are 2 intersections to the circle
     */
    fun splineToCircle(circle: Circle, line: Line, reference: Vector2d, offset: Pose2d = Pose2d()) : TrajectorySequenceBuilder<T> {
        val endPose = Coordinate.lineCircleIntersection(circle, Coordinate.toPoint(line.start), Coordinate.toPoint(line.end)).minByOrNull { it.distTo(reference) }
        return this.splineTo((endPose ?: reference) + offset.vec(), (endPose?.angleTo(circle.center) ?: reference.angleTo(circle.center)) + offset.heading)
    }

    fun splineTo(endPosition: Vector2d, endTangent: Vector2d, offset: Pose2d = Pose2d()) = splineTo(endPosition + offset.vec(), endPosition.angleTo(endTangent) + offset.heading)

    fun splineTo(
        endPosition: Vector2d,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineTo(
                    endPosition.flip(isFlipped), endHeading.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun splineToConstantHeading(
        endPosition: Vector2d,
        endHeading: Double
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToConstantHeading(
                    endPosition.flip(isFlipped), endHeading.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun splineToConstantHeading(
        endPosition: Vector2d,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToConstantHeading(
                    endPosition.flip(isFlipped), endHeading.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun splineToLinearHeading(endPose: Pose2d, endHeading: Double): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToLinearHeading(
                    endPose.flip(isFlipped), endHeading.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun splineToLinearHeading(
        endPose: Pose2d,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToLinearHeading(
                    endPose.flip(isFlipped), endHeading.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    fun splineToSplineHeading(endPose: Pose2d, endHeading: Double): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToSplineHeading(
                    endPose.flip(isFlipped), endHeading.flip(isFlipped), currentVelConstraint, currentAccelConstraint
                )
            }
        })
    }

    fun splineToSplineHeading(
        endPose: Pose2d,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder<T> {
        return addPath(object : AddPathCallback {
            override fun run() {
                currentTrajectoryBuilder?.splineToSplineHeading(
                    endPose.flip(isFlipped), endHeading.flip(isFlipped), velConstraint, accelConstraint
                )
            }
        })
    }

    private fun addPath(callback: AddPathCallback): TrajectorySequenceBuilder<T> {
        if (currentTrajectoryBuilder == null) newPath()
        try {
            callback.run()
        } catch (e: PathContinuityViolationException) {
            newPath()
            callback.run()
        }
        val builtTraj = currentTrajectoryBuilder!!.build()
        val durationDifference = builtTraj.duration() - lastDurationTraj
        val displacementDifference = builtTraj.path.length() - lastDisplacementTraj
        lastPose = builtTraj.end()
        currentDuration += durationDifference
        currentDisplacement += displacementDifference
        lastDurationTraj = builtTraj.duration()
        lastDisplacementTraj = builtTraj.path.length()
        return this
    }

    fun setTangent(tangent: Double): TrajectorySequenceBuilder<T> {
        setAbsoluteTangent = true
        absoluteTangent = tangent
        pushPath()
        return this
    }

    private fun setTangentOffset(offset: Double): TrajectorySequenceBuilder<T> {
        setAbsoluteTangent = false
        tangentOffset = offset
        pushPath()
        return this
    }

    fun setReversed(reversed: Boolean): TrajectorySequenceBuilder<T> {
        return if (reversed) setTangentOffset(Math.toRadians(180.0)) else setTangentOffset(0.0)
    }

    fun setConstraints(
        velConstraint: TrajectoryVelocityConstraint,
        accelConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder<T> {
        currentVelConstraint = velConstraint
        currentAccelConstraint = accelConstraint
        return this
    }

    fun resetConstraints(): TrajectorySequenceBuilder<T> {
        currentVelConstraint = baseVelConstraint
        currentAccelConstraint = baseAccelConstraint
        return this
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint): TrajectorySequenceBuilder<T> {
        currentVelConstraint = velConstraint
        return this
    }

    fun resetVelConstraint(): TrajectorySequenceBuilder<T> {
        currentVelConstraint = baseVelConstraint
        return this
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint): TrajectorySequenceBuilder<T> {
        currentAccelConstraint = accelConstraint
        return this
    }

    fun resetAccelConstraint(): TrajectorySequenceBuilder<T> {
        currentAccelConstraint = baseAccelConstraint
        return this
    }

    fun setTurnConstraint(maxAngVel: Double, maxAngAccel: Double): TrajectorySequenceBuilder<T> {
        currentTurnConstraintMaxAngVel = maxAngVel
        currentTurnConstraintMaxAngAccel = maxAngAccel
        return this
    }

    fun resetTurnConstraint(): TrajectorySequenceBuilder<T> {
        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel
        return this
    }

    fun addTemporalMarker(callback: MarkerCallback?): TrajectorySequenceBuilder<T> {
        return this.addTemporalMarker(currentDuration, callback)
    }

    fun UNSTABLE_addTemporalMarkerOffset(
        offset: Double,
        callback: MarkerCallback?
    ) = addTemporalMarker(currentDuration + offset, callback)

    fun addTemporalMarker(time: Double, callback: MarkerCallback?) = addTemporalMarker(0.0, time, callback)

    fun addTemporalMarker(
        scale: Double,
        offset: Double,
        callback: MarkerCallback?
    ) = addTemporalMarker({ time: Double -> scale * time + offset }, callback)

    fun addTemporalMarker(
        time: TimeProducer?,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder<T> {
        temporalMarkers.add(TemporalMarker(time!!, callback!!))
        return this
    }

    fun addSpatialMarker(point: Vector2d?, callback: MarkerCallback?): TrajectorySequenceBuilder<T> {
        spatialMarkers.add(SpatialMarker(point!!, callback!!))
        return this
    }

    fun addDisplacementMarker(callback: MarkerCallback?): TrajectorySequenceBuilder<T> {
        return this.addDisplacementMarker(currentDisplacement, callback)
    }

    fun UNSTABLE_addDisplacementMarkerOffset(
        offset: Double,
        callback: MarkerCallback?
    ) = addDisplacementMarker(currentDisplacement + offset, callback)

    fun addDisplacementMarker(
        displacement: Double,
        callback: MarkerCallback?
    ) = addDisplacementMarker(0.0, displacement, callback)

    fun addDisplacementMarker(
        scale: Double,
        offset: Double,
        callback: MarkerCallback?
    ) = addDisplacementMarker(
            { displacement: Double -> scale * displacement + offset },
            callback
        )

    fun addDisplacementMarker(
        displacement: DisplacementProducer?,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder<T> {
        displacementMarkers.add(DisplacementMarker(displacement!!, callback!!))
        return this
    }

    @JvmOverloads
    fun turn(
        angle: Double,
        maxAngVel: Double = currentTurnConstraintMaxAngVel,
        maxAngAccel: Double = currentTurnConstraintMaxAngAccel
    ): TrajectorySequenceBuilder<T> {
        pushPath()
        val turnProfile = generateSimpleMotionProfile(
            MotionState(lastPose.heading, 0.0, 0.0, 0.0),
            MotionState(lastPose.heading + angle, 0.0, 0.0, 0.0),
            maxAngVel,
            maxAngAccel
        )
        sequenceSegments.add(TurnSegment(lastPose, angle, turnProfile, emptyList()))
        lastPose = Pose2d(
            lastPose.x, lastPose.y,
            norm(lastPose.heading + angle)
        )
        currentDuration += turnProfile.duration()
        return this
    }

    @JvmOverloads
    fun turnTo(
        absoluteAngle: Double,
        maxAngVel: Double = currentTurnConstraintMaxAngVel,
        maxAngAccel: Double = currentTurnConstraintMaxAngAccel
    ): TrajectorySequenceBuilder<T> {
        return turn(absoluteAngle - lastPose.heading, maxAngVel, maxAngAccel)
    }

    fun waitSeconds(seconds: Double): TrajectorySequenceBuilder<T> {
        pushPath()
        sequenceSegments.add(WaitSegment(lastPose, seconds, emptyList()))
        currentDuration += seconds
        return this
    }

    fun waitSeconds(seconds: Double, driveSignal: DriveSignal): TrajectorySequenceBuilder<T> {
        pushPath()
        sequenceSegments.add(WaitSegment(lastPose, seconds, emptyList(), driveSignal))
        currentDuration += seconds
        return this
    }

    fun waitWhile(condition: () -> Boolean) : TrajectorySequenceBuilder<T> {
        waitSeconds(0.01)
        pushPath()
        sequenceSegments.add(ConditionalWait(lastPose, emptyList(), condition))
        return this
    }

    fun waitWhile(driveSignal: (Double) -> DriveSignal, condition: () -> Boolean) : TrajectorySequenceBuilder<T> {
        waitSeconds(0.01)
        pushPath()
        sequenceSegments.add(ConditionalWait(lastPose, emptyList(), condition, driveSignal))
        return this
    }

    fun performAction(markerCallback: MarkerCallback) : TrajectorySequenceBuilder<T> {
        pushPath()
        sequenceSegments.add(ActionSegment(lastPose, emptyList(), markerCallback))
        return this
    }
    
    fun addTrajectory(trajectory: Trajectory): TrajectorySequenceBuilder<T> {
        pushPath()
        sequenceSegments.add(
            TrajectorySegment(
                trajectory
            )
        )
        return this
    }

    fun addFutureTrajectory(future: FutureSegment, endPose: Pose2d): TrajectorySequenceBuilder<T> {
        future.startPose = lastPose
        future.endPose = endPose
        futurePath(future)
        sequenceSegments.add(
            future
        )
        return this
    }

    fun setState(state: T): TrajectorySequenceBuilder<T> {
        this.state = state
        return this
    }

    /**
     * reflect path across the x-axis
     */
    fun flipSide(): TrajectorySequenceBuilder<T> {
        isFlipped = true
        return this
    }

    private fun pushPath() {
        if (currentTrajectoryBuilder != null) {
            val builtTraj = currentTrajectoryBuilder!!.build()
            sequenceSegments.add(TrajectorySegment(builtTraj, state))
           // state = null
        }
        currentTrajectoryBuilder = null
    }

    private fun newPath() {
        if (currentTrajectoryBuilder != null) pushPath()
        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0
        val tangent = if (setAbsoluteTangent) absoluteTangent else norm(lastPose.heading + tangentOffset)
        currentTrajectoryBuilder = TrajectoryBuilder(
            lastPose,
            tangent,
            currentVelConstraint,
            currentAccelConstraint,
            resolution
        )
    }

    private fun futurePath(future: FutureSegment) {
        if (currentTrajectoryBuilder != null) pushPath()
        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0
        val tangent =
            if (setAbsoluteTangent) absoluteTangent else norm(lastPose.heading + tangentOffset)
        currentTrajectoryBuilder = TrajectoryBuilder(
            future.endPose,
            tangent,
            currentVelConstraint,
            currentAccelConstraint,
            resolution
        )
    }

    fun build(): TrajectorySequence {
        pushPath()
        val globalMarkers = convertMarkersToGlobal(
            sequenceSegments,
            temporalMarkers, displacementMarkers, spatialMarkers
        )
        return TrajectorySequence(
            projectGlobalMarkersToLocalSegments(
                globalMarkers,
                sequenceSegments
            )
        )
    }

    private fun convertMarkersToGlobal(
        sequenceSegments: List<SequenceSegment?>,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): List<TrajectoryMarker> {
        val trajectoryMarkers = ArrayList<TrajectoryMarker>()

        // Convert temporal markers
        for ((producer, callback) in temporalMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(producer.produce(currentDuration), callback)
            )
        }

        // Convert displacement markers
        for ((producer, callback) in displacementMarkers) {
            val time = displacementToTime(
                sequenceSegments,
                producer.produce(currentDisplacement)
            )
            trajectoryMarkers.add(
                TrajectoryMarker(
                    time,
                    callback
                )
            )
        }

        // Convert spatial markers
        for ((point, callback) in spatialMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(
                    pointToTime(sequenceSegments, point),
                    callback
                )
            )
        }
        return trajectoryMarkers
    }

    private fun projectGlobalMarkersToLocalSegments(
        markers: List<TrajectoryMarker>,
        sequenceSegments: MutableList<SequenceSegment?>
    ): List<SequenceSegment?> {
        if (sequenceSegments.isEmpty()) return emptyList()
        var totalSequenceDuration = 0.0
        for (segment in sequenceSegments) {
            totalSequenceDuration += segment!!.duration()
        }
        for ((time, callback) in markers) {
            var segment: SequenceSegment? = null
            var segmentIndex = 0
            var segmentOffsetTime = 0.0
            var currentTime = 0.0
            for (i in sequenceSegments.indices) {
                val seg = sequenceSegments[i]
                val markerTime = min(time, totalSequenceDuration)
                if (seg != null) {
                    if (currentTime + seg.duration() >= markerTime) {
                        segment = seg
                        segmentIndex = i
                        segmentOffsetTime = markerTime - currentTime
                        break
                    } else {
                        currentTime += seg.duration()
                    }
                }
            }
            var newSegment: SequenceSegment? = null
            when (segment) {
                is WaitSegment -> {
                    val newMarkers: MutableList<TrajectoryMarker> = ArrayList(segment.markers)
                    newMarkers.addAll(sequenceSegments[segmentIndex]!!.markers)
                    newMarkers.add(TrajectoryMarker(segmentOffsetTime, callback))
                    val thisSegment = segment
                    newSegment = WaitSegment(thisSegment.startPose, thisSegment.duration(), newMarkers, thisSegment.driveSignal)
                }
                is TurnSegment -> {
                    val newMarkers: MutableList<TrajectoryMarker> = ArrayList(segment.markers)
                    newMarkers.addAll(sequenceSegments[segmentIndex]!!.markers)
                    newMarkers.add(TrajectoryMarker(segmentOffsetTime, callback))
                    val thisSegment = segment
                    newSegment = TurnSegment(
                        thisSegment.startPose,
                        thisSegment.totalRotation,
                        thisSegment.motionProfile,
                        newMarkers
                    )
                }
                is TrajectorySegment -> {
                    val thisSegment = segment
                    val newMarkers: MutableList<TrajectoryMarker> =
                        ArrayList(thisSegment.trajectory.markers)
                    newMarkers.add(TrajectoryMarker(segmentOffsetTime, callback))
                    newSegment = TrajectorySegment(
                        Trajectory(
                            thisSegment.trajectory.path,
                            thisSegment.trajectory.profile,
                            newMarkers
                        ),
                        segment.state
                    )
                }
//                is ConditionalWait -> {
//                    val newMarkers: MutableList<TrajectoryMarker> = ArrayList(segment.markers)
//                    newMarkers.addAll(sequenceSegments[segmentIndex]!!.markers)
//                    newMarkers.add(TrajectoryMarker(segmentOffsetTime, callback))
//                    val thisSegment = segment
//                    newSegment = ConditionalWait(thisSegment.startPose, newMarkers, thisSegment.condition, thisSegment.driveSignal)
//                }
            }
            sequenceSegments[segmentIndex] = newSegment
        }
        return sequenceSegments
    }

    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // note: this assumes that the profile position is monotonic increasing
    private fun motionProfileDisplacementToTime(profile: MotionProfile, s: Double): Double {
        var tLo = 0.0
        var tHi = profile.duration()
        while (abs(tLo - tHi) >= 1e-6) {
            val tMid = 0.5 * (tLo + tHi)
            if (profile[tMid].x > s) {
                tHi = tMid
            } else {
                tLo = tMid
            }
        }
        return 0.5 * (tLo + tHi)
    }

    private fun displacementToTime(sequenceSegments: List<SequenceSegment?>, s: Double): Double {
        var currentTime = 0.0
        var currentDisplacement = 0.0
        for (segment in sequenceSegments) {
            if (segment is TrajectorySegment) {
                val thisSegment = segment
                val segmentLength = thisSegment.trajectory.path.length()
                if (currentDisplacement + segmentLength > s) {
                    val target = s - currentDisplacement
                    val timeInSegment = motionProfileDisplacementToTime(
                        thisSegment.trajectory.profile,
                        target
                    )
                    return currentTime + timeInSegment
                } else {
                    currentDisplacement += segmentLength
                    currentTime += thisSegment.trajectory.duration()
                }
            } else {
                currentTime += segment!!.duration()
            }
        }
        return 0.0
    }

    private fun pointToTime(sequenceSegments: List<SequenceSegment?>, point: Vector2d): Double {
        class ComparingPoints(
            val distanceToPoint: Double,
            val totalDisplacement: Double,
            val thisPathDisplacement: Double
        )

        val projectedPoints: MutableList<ComparingPoints> = ArrayList()
        for (segment in sequenceSegments) {
            if (segment is TrajectorySegment) {
                val thisSegment = segment
                val displacement = thisSegment.trajectory.path.project(point, 0.25)
                val projectedPoint = thisSegment.trajectory.path[displacement].vec()
                val distanceToPoint = point.minus(projectedPoint).norm()
                var totalDisplacement = 0.0
                for (comparingPoint in projectedPoints) {
                    totalDisplacement += comparingPoint.totalDisplacement
                }
                totalDisplacement += displacement
                projectedPoints.add(
                    ComparingPoints(
                        distanceToPoint,
                        displacement,
                        totalDisplacement
                    )
                )
            }
        }
        var closestPoint: ComparingPoints? = null
        for (comparingPoint in projectedPoints) {
            if (closestPoint == null) {
                closestPoint = comparingPoint
                continue
            }
            if (comparingPoint.distanceToPoint < closestPoint.distanceToPoint) closestPoint =
                comparingPoint
        }
        return displacementToTime(sequenceSegments, closestPoint!!.thisPathDisplacement)
    }

    private interface AddPathCallback {
        fun run()
    }

    init {
        currentVelConstraint = baseVelConstraint
        currentAccelConstraint = baseAccelConstraint
        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel
        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel
        sequenceSegments = ArrayList()
        temporalMarkers = ArrayList()
        displacementMarkers = ArrayList()
        spatialMarkers = ArrayList()
        lastPose = startPose
        tangentOffset = 0.0
        setAbsoluteTangent = startTangent != null
        absoluteTangent = startTangent ?: 0.0
        currentTrajectoryBuilder = null
        currentDuration = 0.0
        currentDisplacement = 0.0
        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0
    }
}