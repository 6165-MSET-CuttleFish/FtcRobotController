package com.example.meepmeepsequences.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.example.meepmeepsequences.util.Context.alliance
import com.example.meepmeepsequences.util.Context.location
import com.example.meepmeepsequences.util.Context.side
import com.example.meepmeepsequences.util.geometry.Circle

object FrequentPositions {
    @JvmStatic
    val allianceHub: Circle
        get() = Circle(
            Vector2d(
                -11.0,
                -24.0
            ).flip(alliance == Alliance.BLUE), 19.0
        )

    @JvmStatic
    val carouselVec: Circle
        get() = Circle(
            Vector2d(
                -70.5,
                -70.5
            ).flip(alliance == Alliance.BLUE), 20.0
        )

    @JvmStatic
    fun startingPosition(): Pose2d {
        val regular = if (side == Side.CYCLING) Pose2d(8.2, -58.0, Math.toRadians(-90.0)) else Pose2d(
            -36.0,
            -58.0,
            Math.toRadians(-90.0)
        )
        return regular.flip(alliance == Alliance.BLUE)
    }

    fun dumpPosition(): Pose2d {
        val regular =
            if (side == Side.CYCLING) Pose2d(5.0, -30.0, Math.toRadians(-15.0)) else Pose2d(
                -28.0,
                -31.0,
                Math.toRadians(20.0)
            )
        return regular.flip(alliance == Alliance.BLUE)
    }

    fun cycleDumpPosition(): Pose2d {
        val regular =
            if (side == Side.CYCLING) Pose2d(6.0, -32.0, Math.toRadians(-30.0)) else Pose2d()
        return regular.flip(alliance == Alliance.BLUE)
    }

    fun duckLocation(): Pose2d {
        val arr = duckLocations
        return when (location) {
            Detector.Location.LEFT -> arr[0]
            Detector.Location.RIGHT -> arr[2]
            Detector.Location.MIDDLE -> arr[1]
        }
    }

    fun duckLocation(location: Detector.Location): Pose2d {
        val arr = duckLocations
        return when (location) {
            Detector.Location.LEFT -> arr[0]
            Detector.Location.RIGHT -> arr[2]
            Detector.Location.MIDDLE -> arr[1]
        }
    }

    val barcode : Array<Pose2d>
    get() = if (side == Side.CAROUSEL) arrayOf(
        Pose2d(-27.0, -35.0, Math.toRadians(-30.0)),
        Pose2d(-35.0, -35.0, Math.toRadians(0.0)),
        Pose2d(-44.0, -35.0, Math.toRadians(30.0)),
    ) else arrayOf(
        Pose2d(3.5, -35.0, Math.toRadians(-30.0)),
        Pose2d(12.0, -35.0, Math.toRadians(0.0)),
        Pose2d(20.0, -35.0, Math.toRadians(30.0)),
    )

    private val duckLocations: Array<Pose2d>
        get() {
            return if (alliance == Alliance.RED) {
                if (side == Side.CAROUSEL) {
                    arrayOf(
                        Pose2d(
                            -32.0,
                            -44.0,
                            Math.toRadians(8.0)
                        ),
                        Pose2d(
                            -32.0,
                            -44.0,
                            Math.toRadians(15.0)
                        ),
                        Pose2d(
                            -32.0,
                            -44.0,
                            Math.toRadians(30.0)
                        )
                    )
                } else {
                    arrayOf(
                        Pose2d(
                            1.0,
                            -44.0,
                            Math.toRadians(0.0)
                        ),
                        Pose2d(
                            6.6,
                            -44.0,
                            Math.toRadians(0.0)
                        ),
                        Pose2d(
                            9.0,
                            -44.0,
                            Math.toRadians(-25.0)
                        )
                    )
                }
            } else {
                if (side == Side.CAROUSEL) {
                    arrayOf(
                        Pose2d(
                            -32.0,
                            44.0,
                            Math.toRadians(0.0)
                        ),
                        Pose2d(
                            -40.0,
                            44.0,
                            Math.toRadians(0.0)
                        ),
                        Pose2d(
                            -50.0,
                            44.0,
                            Math.toRadians(0.0)
                        )
                    )
                } else {
                    arrayOf(
                        Pose2d(9.9, 44.0, Math.toRadians(0.0)),
                        Pose2d(
                            9.1,
                            46.0,
                            Math.toRadians(25.0)
                        ).polarAdd(1.0),
                        Pose2d(
                            1.0,
                            44.0,
                            Math.toRadians(-25.0)
                        )
                    )
                }
            }
        }
}