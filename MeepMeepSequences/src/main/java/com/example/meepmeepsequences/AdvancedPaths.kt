package com.example.meepmeepsequences

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.*
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.FrequentPositions.allianceHub
import com.example.meepmeepsequences.util.FrequentPositions.barcode
import com.example.meepmeepsequences.util.FrequentPositions.carouselVec
import com.example.meepmeepsequences.util.FrequentPositions.startingPosition
import com.example.meepmeepsequences.util.geometry.Line
import com.example.meepmeepsequences.util.geometry.flip
import com.example.meepmeepsequences.util.geometry.polarAdd
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity

class AdvancedPaths {
    private fun colorSchemeVariable() = if (alliance == Alliance.BLUE) ColorSchemeBlueDark() else ColorSchemeRedDark()
    private val capstone = Capstone()
    private val deposit = Deposit()
    private val intake = Intake()
    private val carousel = Carousel()
    private val relocalizer = Relocalizer()
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
                        .capstoneReady(capstone)
                        .splineToVectorOffset(barcode[1].vec().flip(blue), Pose2d(14.0, -8.0), (Math.PI / 2 + barcode[1].heading).flip(blue))
                        .capstonePickup(capstone)
                        .liftUp(deposit, Robot.getLevel(location))
                        .waitWhile(capstone::isDoingWork) // capstone loaded
                        .splineToCircle(allianceHub, Line.yAxis(-33.0).flip(blue), Vector2d(-20.0, -24.0).flip(blue))
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .UNSTABLE_addDisplacementMarkerOffset(1.0, carousel::on)
                        .splineTo(carouselVec.center.polarAdd(carouselVec.radius, Math.toRadians(40.0).flip(blue)), carouselVec.center)
                        .waitSeconds(1.2, DriveSignal(Pose2d(5.0), Pose2d(5.0)))
                        .carouselOff(carousel) // drop the ducky
                        .turn(Math.toRadians(45.0))
                        .intakeOff(intake)
                        .setReversed(true)
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineToCircle(allianceHub, Line.yAxis(-19.0).flip(blue), Vector2d(-23.0, -10.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork)
                        .setReversed(false)
                        .forward(3.0)
                        .splineTo(Vector2d(-12.0, -4.0).flip(blue), Math.toRadians(0.0).flip(blue))
                        .lineToSplineHeading(Pose2d(-10.0, -4.0, Math.toRadians(0.0)).flip(blue))
                        .splineTo(Vector2d(10.0, -30.0).flip(blue), Math.toRadians(-90.0).flip(blue))
                for (i in 1..4)
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(10.0) {
                            intake.setPower(1.0)
                        }
                        .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                        .increaseGains()
                        .splineToConstantHeading(Vector2d(28.0, -40.0).flip(blue), 0.0)
                        .defaultGains()
                        .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                        .splineTo(Vector2d(50.0, -45.0).flip(blue), Math.toRadians(-30.0 - 20 * Math.random()).flip(blue))
                        .setReversed(true)
                        .intakeOff(intake)
                        .splineTo(Vector2d(39.0, -40.0).flip(blue), Math.PI)
                        .increaseGains()
                        .splineToConstantHeading(Vector2d(20.0, -40.0).flip(blue), Math.PI)
                        .defaultGains()
                        .liftUp(deposit, Deposit.State.LEVEL3)
                        .splineToCircle(allianceHub, Line.yAxis(-33.0).flip(blue), Vector2d(12.0, -24.0).flip(blue))
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                        .setReversed(false)
                trajectoryBuilder
                    .splineTo(Vector2d(20.0, -40.0).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(39.0, -40.0).flip(blue), 0.0)
                    .build()
            }
    }
    @JvmField var line = -44.0
    @JvmField var coast = -40.5
    @JvmField var intakeY = -45.0
    @JvmField var stop = 51.0
    @JvmField var intakeDelay = 9.0
    @JvmField var conjoiningPoint = 27.0
    @JvmField var waitTime = 0.01
    @JvmField var gainsPoint = 36.0
    @JvmField var depositDistance = 23.0
    @JvmField var divConstant = 1.3
    @JvmField var depositingAngle = -40.0
    @JvmField var intakingAngle = -60.0
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
                        .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                           // Deposit.allowLift = true
                        }
                        .splineTo(allianceHub.center.polarAdd(depositDistance, Math.toRadians(
                            depositingAngle).flip(blue)), allianceHub.center)
                        .setReversed(false)
                        .dump(deposit)
                        .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                for (i in 1..6) {
                    trajectoryBuilder
                        .UNSTABLE_addDisplacementMarkerOffset(intakeDelay) {
                            intake.setPower(1.0)
                        }
                        //.setState(PathState.INTAKING)
                        .splineTo(Vector2d(conjoiningPoint, coast).flip(blue), 0.0)
                        //.increaseGains(GainMode.FORWARD)
                        //.carouselOn(carousel)
                        .splineToConstantHeading(Vector2d(gainsPoint, coast).flip(blue), 0.0)
                        .defaultGains()
                        //.carouselOff(carousel)
                        .splineTo(
                            Vector2d(stop + i / divConstant, intakeY - i / 2).flip(blue),
                            Math.toRadians(intakingAngle - 20 * Math.random()).flip(blue)
                        )
//                .waitWhile(::signalTurn) {
//                    intake.state == Intake.State.OUT
//                }
                        .waitSeconds(waitTime)
                        .setReversed(true)
                        .relocalize(robot)
                        .intakeOff(intake)

                trajectoryBuilder
                    //.setState(PathState.DUMPING)
                    .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                       // Deposit.allowLift = true
                    }
                    .splineTo(Vector2d(39.0, coast).flip(blue), Math.PI)
                    .splineToConstantHeading(Vector2d(gainsPoint, coast).flip(blue), Math.PI)
                    //.increaseGains(Robot.GainMode.BACKWARD)
                    .splineToConstantHeading(Vector2d(conjoiningPoint, coast).flip(blue), Math.PI)
                    //.defaultGains()
                    //.liftUp(deposit, Deposit.State.LEVEL3)
                    .splineTo(
                        allianceHub.center.polarAdd(
                            depositDistance,
                            Math.toRadians(depositingAngle).flip(blue)
                        ),
                        allianceHub.center
                    )
                    .dump(deposit)
                    .waitWhile(deposit::isDoingWork) // wait for platform to dumpPosition
                    .setReversed(false)
            }
                trajectoryBuilder
                    .splineTo(Vector2d(20.0, -55.0).flip(blue), 0.0)
                    .splineToConstantHeading(Vector2d(39.0, -55.0).flip(blue), 0.0)
                    .build()
            }
    }
}