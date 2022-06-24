package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit
import org.firstinspires.ftc.teamcode.util.field.Freight
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableMotor
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import org.firstinspires.ftc.teamcode.roadrunnerext.geometry.polarAdd
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.controllers.LowPassFilter
import org.firstinspires.ftc.teamcode.util.controllers.MovingMedian
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.freight
import org.firstinspires.ftc.teamcode.util.field.Context.opModeType
import org.firstinspires.ftc.teamcode.util.field.Context.side
import org.firstinspires.ftc.teamcode.util.field.OpModeType
import org.firstinspires.ftc.teamcode.util.field.Side
import java.lang.Exception
import kotlin.math.abs
import kotlin.math.sin

/**
 * Frontal mechanism for collecting freight
 * @author Ayush Raman
 */
@Config
class Intake(hardwareMap: HardwareMap) : Module<Intake.State>(hardwareMap, State.IN, Pose2d(7.7)) {
    companion object {
        @JvmField
        var raisedPosition = 0.1
        @JvmField
        var loweredPosition = 1.0
        @JvmField
        var intakeLimit = 14.0
        @JvmField
        var outPosition = 0.41
        @JvmField
        var inPosition = 0.1
        @JvmField
        var midPosition = 0.38
        @JvmField
        var stallingSpeed = 0.9
        @JvmField
        var extensionPositionPerSecond = 0.6
        @JvmField
        var dropPositionPerSecond = 3.0
        @JvmField
        var blueTolerance = 0.01
        @JvmField
        var smoothingCoeffecientDistance = 0.7
        @JvmField
        var smoothingCoefficientAlpha = 0.8
        @JvmField
        var div = 1.0
        @JvmField
        var distanceTolerance = 9.0
        @JvmField
        var transferTolerance = 6.0
    }
    enum class State(override val timeOut: Double? = null) : StateBuilder {
        OUT,
        TRANSFER(1.0),
        IN,
        CREATE_CLEARANCE,
        COUNTER_BALANCE,
        STEP_BRO,
    }

    private var intake = ControllableMotor(hardwareMap.get(DcMotorEx::class.java, "intake"))
    private var extensionServos =
        ControllableServos(
            hardwareMap.servo["outL"],
            hardwareMap.servo["outR"],
            hardwareMap.servo["outA"],
        )
    private var flip = ControllableServos(hardwareMap.servo["flip"])
    private var blockSensor = hardwareMap.get(ColorRangeSensor::class.java, "block")
    private var extensionDistance = hardwareMap.get(ColorRangeSensor::class.java, "extDistance")
    private var power = 0.0
    var containsBlock = false
    private var distanceFilter = LowPassFilter(smoothingCoeffecientDistance, 0.0)
    private var colorFilter = LowPassFilter(smoothingCoefficientAlpha, 0.0)
    private var distanceMedian = MovingMedian(3)
    override fun internalInit() {
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        flip.positionPerSecond = dropPositionPerSecond
        extensionServos.positionPerSecond = extensionPositionPerSecond
        extensionServos.setLimits(inPosition, 0.45)
        if (opModeType == OpModeType.TELE) {
            flip.init(0.5)
            flip.init(0.5)
            flip.init(0.5)
            extensionServos.init(midPosition)
        } else {
//            flip.init(raisedPosition)
//            extensionServos.init(inPosition)
        }
        setActuators(flip, intake)
    }
    private var shortIntake = false
    var dontFlipOut = false

    fun setPower(power: Double) {
        if (this.power > 0 && power <= 0 || this.power < 0 && power >= 0 || this.power == 0.0 && power != 0.0) {
            if (power != 0.0 && !isDoingWork) {
                state = State.OUT
                Deposit.isLoaded = false
            } else if (isDoingWork) {
                state = State.IN
            }
        }
        shortIntake = power > 1
        this.power = power
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    override fun isDoingInternalWork(): Boolean = state == State.OUT || state == State.TRANSFER

    public override fun internalUpdate() {
        flip.positionPerSecond = dropPositionPerSecond
        extensionServos.positionPerSecond = extensionPositionPerSecond
        distanceFilter.a = smoothingCoeffecientDistance
        colorFilter.a = smoothingCoefficientAlpha
        var power = power
        val unfilteredBlue = color
        val blue = colorFilter.update(unfilteredBlue)
        val unfilteredDistance = distance
        val filteredDistance = distanceFilter.update(unfilteredDistance)
        val medianDistance = if (opModeType == OpModeType.TELE) filteredDistance else distanceMedian.update(unfilteredDistance)
        when (state) {
            State.OUT -> {
                deploy()
                if (containsBlock && !flip.isTransitioning) {
                    state = State.IN
                }
                if (flip.error > 0.3) {
                    power = 0.0
                }
                containsBlock = medianDistance < intakeLimit
            }
            State.IN -> {
                retract()
                if (medianDistance < intakeLimit && isTransitioningState() && previousState == State.OUT) {
                    containsBlock = true
                }
                if (containsBlock) {
                    if (isTransitioningState()) {
                        freight = if (blue > blueTolerance) {
                            Freight.BALL
                        } else {
                            Freight.CUBE
                        }
                    } else {
                        if (extensionDistance.getDistance(DistanceUnit.CM) < distanceTolerance) {
                            state = State.TRANSFER
                        }
                    }
                }
                if (isTransitioningState()) {
                    power = stallingSpeed
                }
            }
            State.TRANSFER -> {
                power = -1.0
                containsBlock = false
                // dontFlipOut = false
                val minTime = if (freight == Freight.BALL) 0.4 else 0.3
                if (filteredDistance > transferTolerance && secondsSpentInState > minTime) {
                    Deposit.isLoaded = true
                }
                if (Deposit.isLoaded && opModeType == OpModeType.AUTO && side == Side.CAROUSEL) {
                    flip.position = 0.5
                }
                if ((Deposit.isLoaded && secondsSpentInState > (state.timeOut?.div(div) ?: 0.0)) || secondsSpentInState > (state.timeOut ?: 0.0)) {
                    state = State.IN
                    power = 0.0
                    this.power = power
                }
            }
            State.CREATE_CLEARANCE -> {
                extensionServos.position = midPosition
                flip.position = 0.5
                // raiseIntake()
                if (!extensionServos.isTransitioning) {
                    state = State.IN
                }
            }
            State.COUNTER_BALANCE -> {
                extensionServos.position = midPosition
                raiseIntake()
            }
            State.STEP_BRO -> {
                extensionServos.position = extension
                dropIntake()
                if (containsBlock && !flip.isTransitioning) {
                    state = State.IN
                }
                power = if (flip.error > 0.3) 0.0 else 1.0
                containsBlock = filteredDistance < intakeLimit
            }
        }
        poseOffset = Pose2d(7.7 + extensionServos.estimatedPosition * 6.0)
        intake.power = if (isHazardous) 0.0 else power
        if (isDebugMode) {
            Context.packet.put("containsBlock", containsBlock)
            Context.packet.put("Extension Real Position", extensionServos.estimatedPosition)
            Context.packet.put("Drop Real Position", flip.estimatedPosition)
            Context.packet.put("Raw Blue", unfilteredBlue)
            Context.packet.put("Filtered Blue", blue)
            Context.packet.put("Block Distance Filtered", filteredDistance)
            Context.packet.put("Block Distance Median", medianDistance)
            Context.packet.put("Block Distance Raw", unfilteredDistance)
            Context.packet.put("Extension Distance", extensionDistance.getDistance(DistanceUnit.CM))
        }
        val intakePose = modulePoseEstimate.polarAdd(7.7)
        DashboardUtil.drawIntake(Context.packet.fieldOverlay(), modulePoseEstimate, intakePose)
    }

    private val distance: Double
        get() = try {
                blockSensor?.getDistance(DistanceUnit.CM) ?: 0.0
            } catch (e: Exception) {
                0.0
            }
    private val color: Double
        get() = try {
            blockSensor.normalizedColors?.blue?.toDouble() ?: 0.0
        } catch (e: Exception) {
            0.0
        }

    private fun dropIntake() {
        flip.position = loweredPosition
    }

    private fun raiseIntake() {
        flip.position = if (Deposit.isLoaded || containsBlock || dontFlipOut) raisedPosition else 0.5
    }

    fun createClearance() {
        if (state != State.OUT) state = State.CREATE_CLEARANCE
    }

    fun counterBalance() {
        state = State.COUNTER_BALANCE
    }

    fun retractIntake() {
        if (state != State.TRANSFER) state = State.IN
    }

    private fun deploy() {
        Deposit.isLoaded = false
        dropIntake()
        if (flip.isTransitioning && shortIntake) {
            extensionServos.lock()
        } else {
            slidesOut()
        }
    }

    private fun retract() {
        slidesIn()
        raiseIntake()
    }

    private fun slidesOut() {
        if (isHazardous) return
        extensionServos.position = outPosition
    }

    private fun slidesIn() {
        extensionServos.position = inPosition
    }
    private var extension = outPosition
    fun stepbro(extension: Double) {
        this.extension = extension
        if (!containsBlock && !Deposit.isLoaded && state != State.TRANSFER) state = State.STEP_BRO
    }

    fun stepsis() {
        if (state == State.STEP_BRO) state = State.IN
    }

    override fun isTransitioningState() = extensionServos.isTransitioning || flip.isTransitioning
}