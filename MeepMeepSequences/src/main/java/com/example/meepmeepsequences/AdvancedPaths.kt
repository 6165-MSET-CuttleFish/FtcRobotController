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
                val trajectoryBuilder =
                    robot.trajectorySequenceBuilder(startingPosition())
                        .setReversed(true)
                        .liftUp(deposit, Robot.getLevel(location))
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
                                carouselVec.radius,
                                Math.toRadians(40.0).flip(blue)
                            ), carouselVec.center
                        )
                        .waitSeconds(1.2, DriveSignal(Pose2d(5.0)))
                        .carouselOff(carousel) // drop the ducky
                        .intakeOn(intake)
                        .turn(Math.toRadians(60.0).flip(blue))
                        .turn(Math.toRadians(-120.0).flip(blue))
                        .intakeOff(intake)
                        .setReversed(true)
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineTo(
                            allianceHub.center.polarAdd(
                                cyclingDistance, Math.toRadians(
                                    -125.0
                                ).flip(blue)
                            ), allianceHub.center
                        )
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork)
                        .setReversed(false)
                        .splineTo(Vector2d(-55.0, -46.0).flip(blue), Math.toRadians(90.0).flip(blue))
                        .splineTo(Vector2d(-55.0, -27.0).flip(blue), Math.toRadians(90.0).flip(blue))
                        .splineTo(Vector2d(-12.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                        .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
                for (i in 1..2)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                            intake.setPower(1.0)
                        }
                        .splineTo(Vector2d(16.0, -20.0).flip(blue), Math.toRadians(-40.0).flip(blue))
                        .increaseGains()
                        .splineTo(Vector2d(30.0, polesCoast).flip(blue), Math.toRadians(-40.0).flip(blue))
                        .defaultGains()
                        .splineTo(
                            Vector2d(50.0, -45.0).flip(blue),
                            Math.toRadians(-30.0 - 20 * Math.random()).flip(blue)
                        )
                        .turn(Math.toRadians(30.0).flip(blue))
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(Vector2d(30.0, polesCoast).flip(blue), Math.toRadians(180 - 40.0).flip(blue))
                        .increaseGains()
                        .splineTo(Vector2d(16.0, -20.0).flip(blue), Math.toRadians(180 - 40.0).flip(blue))
                        .defaultGains()
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineTo(
                            allianceHub.center.polarAdd(
                                closeDist, Math.toRadians(40.0).flip(blue)
                            ), allianceHub.center
                        )
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                trajectoryBuilder
                    .splineTo(Vector2d(16.0, -20.0).flip(blue), Math.toRadians(-40.0).flip(blue))
                    .splineTo(Vector2d(50.0, polesCoast).flip(blue), Math.toRadians(0.0).flip(blue))
                    .build()
            }
    }

    @JvmField var coast = -55.5
    @JvmField var stop = 52.0
    @JvmField var intakeDelay = 16.0
    @JvmField var depositDelay = 5.0
    @JvmField var closeDist = 25.0
    @JvmField var conjoiningPoint = 20.0
    @JvmField var conjoiningDeposit = 30.0
    @JvmField var waitTime = 0.08
    @JvmField var gainsPoint = 36.0
    @JvmField var cyclingDistance = 29.0
    @JvmField var divConstant = 6.0
    @JvmField var depositingAngle = -60.0
    @JvmField var cyclingAngle = -55.0
    @JvmField var depositingTimeout = 0.3
    @JvmField var intakeError = 7.0
    @JvmField var depositError = 7.0
    @JvmField var intakeVelo = 30.0
    @JvmField var depositVelo = 60.0
    @JvmField var angleOffset = -6.0
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
                for (i in 1..6) {
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                            intake.setPower(1.0)
                            //deposit.liftDown()
                        }
                        //.setState(PathState.INTAKING)
                        .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), 0.0)
                        //.liftLevel(deposit, Deposit.Level.LEVEL3)
                        .increaseGains(intakeVelo)
                        .splineTo(Vector2d(34.0, coast).flip(blue), 0.0)
                        .defaultGains()
                        .splineTo(
                            Vector2d(stop + i / divConstant, coast).flip(blue) + Vector2d(
                                randomRange(-5.0, 5.0),
                                randomRange(-5.0, 5.0)
                            ).flip(blue),
                            Math.toRadians(randomRange(-20.0, 20.0)).flip(blue)
                        )
//                        .waitWhile(::signalTurn) {
//                            intake.state == Intake.State.OUT
//                        }
                        .waitSeconds(waitTime)
                        .relocalize(robot)
                        .setReversed(true)
                        .intakeOff(intake)
                    trajectoryBuilder
                        //.setState(PathState.DUMPING)
                        .splineTo(Vector2d(39.0, coast).flip(blue), Math.PI) // change
                        .splineToConstantHeading(Vector2d(gainsPoint, coast).flip(blue), Math.PI)
                        //.UNSTABLE_addDisplacementMarkerOffset(depositDelay, deposit::liftUp)
                        .increaseGains(depositVelo)
                        .splineToConstantHeading(Vector2d(conjoiningDeposit, coast).flip(blue), Math.PI)
                        .defaultGains()
                        .splineTo(
                            allianceHub.center.polarAdd(
                                cyclingDistance,
                                Math.toRadians(cyclingAngle).flip(blue)
                            ),
                            allianceHub.center,
                            Pose2d(0.0, 0.0, Math.toRadians(angleOffset).flip(blue))
                        )
                        //.waitWhile(deposit::isTransitioningState)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                }
                trajectoryBuilder
                    //.setState(PathState.IDLE)
                    .splineTo(Vector2d(20.0, coast).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(stop, coast).flip(blue), 0.0)
                    .build()
            }
    }
}