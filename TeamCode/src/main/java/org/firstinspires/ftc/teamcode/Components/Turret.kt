package org.firstinspires.ftc.teamcode.Components

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.util.TurretTuner
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Components.Details.packet
import org.firstinspires.ftc.teamcode.PurePursuit.polarAdd
import org.firstinspires.ftc.teamcode.util.BPIDFController
import kotlin.math.abs

@Config
class Turret(hardwareMap: HardwareMap) : Component {
    var turret: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "turret")
    companion object {
        var ANGLE_PID = PIDCoefficients(8.0, 4.0, 0.03)
        var kV = 0.0
        var kStatic = 0.0
        var kA = 0.0
        var TICKS_PER_REVOLUTION = 28.0
        var GEAR_RATIO = 68.0 / 13.0 * (110.0 / 24.0)
        var TOLERANCE = 0
    }
    private var lastKv = kV
    private var lastKp = ANGLE_PID.kP
    private var lastKi = ANGLE_PID.kI
    private var lastKd = ANGLE_PID.kD
    private var lastKStatic = kStatic
    private var lastKa = kA
    var angleControl: BPIDFController = BPIDFController(ANGLE_PID, 2.0, kV, kA, kStatic)
    private var targetAngle = 0.0
    var batteryVoltageSensor: VoltageSensor
    @JvmField
    var offset = 1.5
    var target: Vector2d? = null
    private val turretTuner: TurretTuner = TurretTuner()
    var state = State.IDLE

    enum class State {
        TARGET_LOCK, TUNING, IDLE
    }

    override fun update() {
        val currHeading = Details.robotPose.heading
        val turretCoord = Details.robotPose.vec().polarAdd(currHeading - Math.PI, 3.0)
        var targetAng = 0.0
        when (state) {
            State.TARGET_LOCK -> {
                targetAng = getClosestAngle(Math.toDegrees(targetAngle - Details.robotPose.heading))
                angleControl.targetVelocity = -Details.poseVelocity.heading
                if (target != null) {
                    targetAng = getClosestAngle(turretCoord.angleBetween(target!!) - Math.toDegrees(Details.robotPose.heading))
                    val nextTurretCoord = Details.getFuturePose().vec().polarAdd(currHeading - Math.PI, 3.0)
                    val getFutureTarget = getClosestAngle(nextTurretCoord.angleBetween(target!!) - Math.toDegrees(Details.getFuturePose().heading))
                    angleControl.targetVelocity = getFutureTarget - targetAng
                }
                targetAng += offset
            }
            State.IDLE -> {
                targetAng = closestZero
                angleControl.targetVelocity = 0.0
                if (Details.opModeType == OpModeType.AUTO) targetAng = 0.0
            }
            State.TUNING -> {
                if (!turretTuner.running) turretTuner.start()
                targetAng = turretTuner.update()
            }
        }
        val upperBound = 380.0
        val lowerBound = -380.0
        if (targetAng > upperBound) {
            targetAng -= 360.0
        } else if (targetAng < lowerBound) {
            targetAng += 360.0
        }
        angleControl.targetPosition = Math.toRadians(targetAng)
        if (turret.getCurrent(CurrentUnit.MILLIAMPS) < 9000) {
            angleControl.reset()
        }
        val currAngle = relativeAngle
        val power = angleControl.update(currAngle, angularVelocity)
        if (absError < TOLERANCE) {
            turret.power = 0.0
        } else {
            turret.power = power
        }
        if (lastKv != kV || lastKa != kA || lastKStatic != kStatic || lastKp != ANGLE_PID.kP || lastKi != ANGLE_PID.kI || lastKd != ANGLE_PID.kD) {
            lastKv = kV
            lastKa = kA
            lastKStatic = kStatic
            lastKp = ANGLE_PID.kP
            lastKi = ANGLE_PID.kI
            lastKd = ANGLE_PID.kD
            setPIDCoefficients()
        }
        packet.put("Turret Angle", Math.toDegrees(currAngle))
        packet.put("Turret Velocity", turret.velocity)
        packet.put("Target Angle", targetAng)
        packet.put("Angle Error", absError)
        DashboardUtil.drawTurret(packet.fieldOverlay(), Pose2d(turretCoord.x, turretCoord.y, absoluteAngle), state == State.TARGET_LOCK)
    }

    private fun setPIDCoefficients() {
        angleControl = BPIDFController(ANGLE_PID, kV * 12 / batteryVoltageSensor.voltage, kA, kStatic)
    }

    private val relativeAngle: Double
        get() = ticksToAngle(turret.currentPosition.toDouble())

    private fun ticksToAngle(ticks: Double): Double {
        return ticks * (2 * Math.PI / (TICKS_PER_REVOLUTION * GEAR_RATIO))
    }

    val absoluteAngle: Double
        get() = Details.robotPose.heading + relativeAngle
    private val angularVelocity: Double
        get() = ticksToAngle(turret.velocity)

    fun angleToTicks(angle: Double): Double {
        return angle / (2 * Math.PI) * (TICKS_PER_REVOLUTION * GEAR_RATIO)
    }

    private fun getClosestAngle(targetAngle: Double): Double {
        val curr = Math.toDegrees(relativeAngle)
        val option1 = if (curr > targetAngle) targetAngle + 360 else targetAngle - 360
        val range1 = abs(option1 - curr)
        val range2 = abs(targetAngle - curr)
        return if (range1 < range2) option1 else targetAngle
    }

    private val closestZero: Double
        get() {
            val curr = Math.toDegrees(relativeAngle)
            val possibilities = doubleArrayOf(-720.0, -360.0, 0.0, 360.0, 720.0)
            var minRange = 360.0
            var index = 0
            for (i in possibilities.indices) {
                val range = abs(possibilities[i] - curr)
                if (range < minRange) {
                    minRange = range
                    index = i
                }
            }
            return possibilities[index]
        }

    fun setTargetAngle(angle: Double) {
        targetAngle = angle
        target = null
        state = State.TARGET_LOCK
    }

    @JvmName("setTarget1")
    fun setTarget(vector2d: Vector2d?) {
        target = vector2d
        state = State.TARGET_LOCK
    }

    val velocity: Double
        get() = turret.velocity
    val error: Double
        get() = Math.toDegrees(angleControl.lastError)
    val absError: Double
        get() = abs(error)
    val isIdle: Boolean
        get() = abs(turret.velocity) < 600 && absError < 4
    val isOnTarget: Boolean
        get() = if (state != State.TARGET_LOCK) false else abs(turret.velocity) <= 50 && absError < 0.9



    init {
        if (Details.opModeType == OpModeType.AUTO) turret.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        setPIDCoefficients()
    }
}