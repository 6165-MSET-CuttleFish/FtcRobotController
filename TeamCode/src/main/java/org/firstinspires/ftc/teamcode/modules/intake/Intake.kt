package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.modules.deposit.Platform
import org.firstinspires.ftc.teamcode.util.field.Freight
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableMotor
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.controllers.LowPassFilter
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.freight
import org.firstinspires.ftc.teamcode.util.field.Context.opModeType
import org.firstinspires.ftc.teamcode.util.field.OpModeType

/**
 * Frontal mechanism for collecting freight
 * @author Ayush Raman
 */
@Config
class Intake(hardwareMap: HardwareMap) : Module<Intake.State>(hardwareMap, State.IN, Pose2d(7.7)) {
    companion object {
        @JvmField
        var raisedPosition = 0.2
        @JvmField
        var loweredPosition = 1.0
        @JvmField
        var intakeLimit = 12.0
        @JvmField
        var outPosition = 0.45
        @JvmField
        var inPosition = 0.1
        @JvmField
        var midPosition = 0.4
        @JvmField
        var stallingSpeed = 0.5
        @JvmField
        var extensionPositionPerSecond = 0.6
        @JvmField
        var dropPositionPerSecond = 3.0
        @JvmField
        var alphaTolerance = 0.1
        @JvmField
        var smoothingCoeffecientDistance = 0.8
        @JvmField
        var smoothingCoefficientAlpha = 0.8
        @JvmField
        var div = 2.0
        @JvmField
        var distanceTolerance = 12.0
    }
    enum class State(override val timeOut: Double? = null) : StateBuilder {
        OUT,
        TRANSFER(0.8),
        IN,
        CREATE_CLEARANCE,
        COUNTER_BALANCE,
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
    private var extensionDistance = hardwareMap.get(Rev2mDistanceSensor::class.java, "extDistance")
    private var power = 0.0
    var containsBlock = false
    private var distanceFilter = LowPassFilter(smoothingCoeffecientDistance, 0.0)
    private var colorFilter = LowPassFilter(smoothingCoefficientAlpha, 0.0)
    override fun internalInit() {
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        // Extend range of servo by 30°
        flip.positionPerSecond = dropPositionPerSecond
        extensionServos.positionPerSecond = extensionPositionPerSecond
        extensionServos.init(inPosition)
        flip.init(raisedPosition)
        flip.init(raisedPosition)
        setActuators(flip, intake)
    }
    private var shortInake = false
    fun setPower(power: Double) {
        if (this.power > 0 && power <= 0 || this.power < 0 && power >= 0 || this.power == 0.0 && power != 0.0) {
            if (power != 0.0 && !isDoingWork) {
                state = State.OUT
            } else if (isDoingWork) {
                state = State.IN
            }
        }
        shortInake = power > 1
        this.power = power
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    override fun isDoingInternalWork(): Boolean {
        return state == State.OUT || state == State.TRANSFER
    }

    public override fun internalUpdate() {
        flip.positionPerSecond = dropPositionPerSecond
        extensionServos.positionPerSecond = extensionPositionPerSecond
        distanceFilter.a = if (state == State.OUT) 0.75 else smoothingCoeffecientDistance
        colorFilter.a = smoothingCoefficientAlpha
        var power = power
        val unfilteredAlpha = blockSensor.normalizedColors.blue.toDouble()
        val alpha = colorFilter.update(unfilteredAlpha)
        val unfilteredDistance = distance
        val distance = distanceFilter.update(unfilteredDistance)
        when (state) {
            State.OUT -> {
                deploy()
                if (containsBlock) {
                    state = State.IN
                }
                containsBlock = distance < intakeLimit
            }
            State.IN -> {
                retract()
                if (distance < intakeLimit && isTransitioningState() && previousState == State.OUT) {
                    containsBlock = true
                }
                if (containsBlock) {
                    if (isTransitioningState()) {
                        freight = if (alpha > alphaTolerance) {
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
                if ((Platform.isLoaded && secondsSpentInState > (state.timeOut?.div(div) ?: 0.0)) || secondsSpentInState > (state.timeOut ?: 0.0)) {
                    state = State.IN
                    power = 0.0
                    this.power = power
                }
            }
            State.CREATE_CLEARANCE -> {
                extensionServos.position = midPosition
                if (!extensionServos.isTransitioning) {
                    state = State.IN
                }
            }
            State.COUNTER_BALANCE -> {
                extensionServos.position = midPosition
            }
        }
        poseOffset = Pose2d(7.7 + extensionServos.realPosition * 6.0)
        intake.power = if (isHazardous) 0.0 else power
        if (isDebugMode) {
            Context.packet.put("containsBlock", containsBlock)
            Context.packet.put("Extension Real Position", extensionServos.realPosition)
            Context.packet.put("Drop Real Position", flip.realPosition)
            Context.packet.put("Raw Blue", unfilteredAlpha)
            Context.packet.put("Filtered Blue", alpha)
            Context.packet.put("Filtered Block Distance", distance)
            Context.packet.put("Block Sensor Distance", unfilteredDistance)
            Context.packet.put("Extension Distance", extensionDistance.getDistance(DistanceUnit.CM))
        }
        val intakePose = modulePoseEstimate.polarAdd(7.7)
        DashboardUtil.drawIntake(Context.packet.fieldOverlay(), modulePoseEstimate, intakePose)
    }

    private val distance: Double
        get() = blockSensor.getDistance(DistanceUnit.CM)

    private fun dropIntake() {
        flip.position = loweredPosition
    }

    private fun raiseIntake() {
        flip.position = raisedPosition
    }

    fun createClearance() {
        state = State.CREATE_CLEARANCE
    }

    fun counterBalance() {
        state = State.COUNTER_BALANCE
    }

    fun retractIntake() {
        if (opModeType != OpModeType.AUTO) state = State.IN
    }

    private fun deploy() {
        Platform.isLoaded = false
        dropIntake()
        if (flip.isTransitioning && shortInake) {
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

    override fun isTransitioningState() = extensionServos.isTransitioning || flip.isTransitioning
}