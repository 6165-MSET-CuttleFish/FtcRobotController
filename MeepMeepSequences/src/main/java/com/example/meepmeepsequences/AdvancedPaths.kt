package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.FrequentPositions.allianceHub
import com.example.meepmeepsequences.util.FrequentPositions.carouselVec
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.geometry.flip
import com.example.meepmeepsequences.util.geometry.polarAdd
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

class AdvancedPaths {
    private fun colorSchemeVariable() =
        if (alliance == Alliance.BLUE) ColorSchemeBlueDark() else ColorSchemeRedDark()

    private val capstone = Capstone()
    private val deposit = Deposit()
    private val intake = Intake()
    private val carousel = Carousel()
    private val relocalizer = Relocalizer()
    @JvmField
    var polesCoast = -35.0
    fun carouselPath(blue: Boolean, meepMeep: MeepMeep): RoadRunnerBotEntity {
        side = Side.CAROUSEL
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return DefaultBotBuilder(meepMeep)
            .setColorScheme(colorSchemeVariable()) // Set theme
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                    robot.trajectorySequenceBuilder(startingPosition())
                        .setReversed(true)
                        .liftLevel(deposit, Robot.getLevel(location))
                        .splineTo(
                            allianceHub.center.polarAdd(
                                cyclingDistance, Math.toRadians(
                                    -125.0
                                ).flip(blue)
                            ), allianceHub.center
                        )
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                        .splineTo(
                            carouselVec.center.polarAdd(
                                carouselVec.radius + 10.0,
                                Math.toRadians(40.0).flip(blue)
                            ), carouselVec.center,
                            Pose2d(0.0, 0.0, Math.toRadians(20.0))
                        )
                        .waitSeconds(2.0, DriveSignal(Pose2d(5.0)))
                        .carouselOff(carousel) // drop the ducky
                        .intakeOn(intake)
                        .back(5.0)
                        .turn(Math.toRadians(60.0).flip(blue))
                        .turn(Math.toRadians(-120.0).flip(blue))
                        .intakeOff(intake)
                        .setReversed(true)
                        .liftLevel(deposit, Deposit.Level.LEVEL3)
                        .splineTo(
                            allianceHub.center.polarAdd(
                                cyclingDistance, Math.toRadians(
                                    -125.0
                                ).flip(blue)
                            ), allianceHub.center,
                        )
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork)
                        .setReversed(false)
                        .splineTo(Vector2d(-55.0, -46.0).flip(blue), Math.toRadians(90.0).flip(blue))
                        .splineTo(Vector2d(-55.0, -27.0).flip(blue), Math.toRadians(90.0).flip(blue))
                    .build()
            }
    }
        @JvmField var coast = -54.5
    @JvmField var closeDist = 25.0
        @JvmField var stop = 51.0
        @JvmField var intakeDelay = 5.0
        @JvmField var powerDelay = 6.5
        @JvmField var depositDelay = 28.0
        @JvmField var cycles = 7
        @JvmField var conjoiningPoint = 25.0
        @JvmField var conjoiningDeposit = 30.0
        @JvmField var waitTime = 0.1
        @JvmField var gainsPoint = 36.0
        @JvmField var cyclingDistance = 22.0
        @JvmField var depositDistance = 25.0
        @JvmField var divConstant = 4.0
        @JvmField var depositingAngle = -50.0
        @JvmField var cyclingAngle = -50.0
        @JvmField var depositingTimeout = 0.4
        @JvmField var intakeError = 8.0
        @JvmField var depositError = 6.0
        @JvmField var intakeCrossingVelo = 27.0
        @JvmField var intakeVelo = 45.0
        @JvmField var intakeAngle = 5.0
        @JvmField var depositVelo = 60.0
        @JvmField var angleOffset = -11.0
        @JvmField var yIncrement = -0.0
        @JvmField var offsetNext = true
        @JvmField var skipNext = true
        @JvmField var intakeMinX = 45.0
    @JvmField var pathRotationOffset = -5.0
    fun randomRange(lower: Double, upper: Double): Double{
        return Math.random() * upper + Math.random() * lower
    }
    fun cyclingPath(blue: Boolean, meepMeep: MeepMeep): RoadRunnerBotEntity {
        side = Side.CYCLING
        alliance = if (blue) Alliance.BLUE else Alliance.RED
        return DefaultBotBuilder(meepMeep)
            .setColorScheme(colorSchemeVariable()) // Set theme
            .configure() // configure robot
            .followTrajectorySequence { robot ->
                val trajectoryBuilder =
                    robot.trajectorySequenceBuilder(startingPosition())
                    .setReversed(true)
                    //.liftLevel(deposit, Deposit.Level.LEVEL3)
                    .splineTo(
                        allianceHub.center.polarAdd(
                            cyclingDistance,
                            Math.toRadians(depositingAngle).flip(blue)),
                        allianceHub.center,
                        Pose2d(0.0, 0.0, Math.toRadians(-angleOffset).flip(blue)),
                    )
                    .setReversed(false)
                    .dump(deposit)
                    .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                var coast = coast
                for (i in 1..7) {
                    val angle = Math.toRadians(randomRange(-intakeAngle, 0.0)).flip(blue)
                    val forwardTangent = Math.toRadians(pathRotationOffset).flip(blue)
                    val conjoiningVec = Vector2d(conjoiningPoint, coast).flip(blue)
                    trajectoryBuilder

                       // .setState(PathState.INTAKING)
                        .splineTo(conjoiningVec, forwardTangent)
                        .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                            intake.setPower(0.1)
                            //deposit.liftDown()
                        }
                        .UNSTABLE_addDisplacementMarkerOffset(powerDelay) {
                            intake.setPower(1.0)
                            //deposit.liftDown()
                        }
                        .liftLevel(deposit, Deposit.Level.LEVEL3)
                        .increaseGains(intakeCrossingVelo)
                        .splineToConstantHeading(conjoiningVec.polarAdd(gainsPoint - conjoiningPoint, forwardTangent), forwardTangent)
                        .increaseGains(intakeVelo)
                        .splineTo(conjoiningVec
                            .polarAdd(gainsPoint - conjoiningPoint, forwardTangent)
                            .polarAdd(stop - gainsPoint + i / divConstant, forwardTangent + angle)
                            ,forwardTangent + angle)
                        .defaultGains()
                        .waitSeconds(waitTime)
                        .relocalize(robot)
                        .setReversed(true)
                        .intakeOff(intake)
                    trajectoryBuilder
                       // .setState(PathState.DUMPING)
                        .increaseGains(depositVelo)
                        .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), Math.PI + forwardTangent) // change
                        .splineTo(
                            allianceHub.center.polarAdd(
                                cyclingDistance,
                                Math.toRadians(cyclingAngle).flip(blue)
                            ),
                            allianceHub.center,
                            Pose2d(0.0, 0.0, Math.toRadians(angleOffset).flip(blue))
                        )
                        .defaultGains()
                        //.waitWhile(deposit::isTransitioningState)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                    coast -= yIncrement
                }
                trajectoryBuilder
                    //.setState(PathState.IDLE)
                    .increaseGains(65.0)
                    .splineTo(Vector2d(20.0, coast).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(stop + 2, coast).flip(blue), 0.0)
                    .build()
            }
    }
}