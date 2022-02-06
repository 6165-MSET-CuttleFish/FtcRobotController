package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.modules.deposit.Platform
import org.firstinspires.ftc.teamcode.modules.wrappers.ControllableMotor
import org.firstinspires.ftc.teamcode.modules.wrappers.ControllableServos
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.field.Context

/**
 * Frontal mechanism for collecting freight
 * @author Ayush Raman
 */
@Config
class Intake(hardwareMap: HardwareMap) : Module<Intake.State>(hardwareMap, State.IN, Pose2d(7.7), 0.6) {
    companion object {
        @JvmField
        var raisedPosition = 0.0
        @JvmField
        var loweredPosition = 0.70
        @JvmField
        var intakeLimit = 8.0
        @JvmField
        var outPosition = 0.39
        @JvmField
        var inPosition = 0.0
        @JvmField
        var midPosition = 0.22
        @JvmField
        var extensionPositionPerSecond = 0.5
        @JvmField
        var dropPositionPerSecond = 2.1
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
    private var power = 0.0
    private var containsBlock = false

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

    fun setPower(power: Double) {
        if (this.power > 0 && power <= 0 || this.power < 0 && power >= 0 || this.power == 0.0 && power != 0.0) {
            if (power != 0.0 && !isDoingWork) {
                state = State.OUT
            } else if (isDoingWork) {
                state = State.IN
            }
        }
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
        var power = power
        when (state) {
            State.OUT -> {
                deploy()
                if (distance < intakeLimit) {
                    state = State.IN
                    containsBlock = true
                }
            }
            State.IN -> {
                retract()
                if (!isTransitioningState() && previousState == State.OUT) {
                    state = if (distance < intakeLimit) State.TRANSFER else State.IN
                }
                if (extensionServos.isTransitioning || flip.isTransitioning) {
                    power = 0.7
                }
            }
            State.TRANSFER -> {
                power = -1.0
                containsBlock = false
                if ((Platform.isLoaded && secondsSpentInState > (state.timeOut?.div(2) ?: 0.0)) || secondsSpentInState > (state.timeOut ?: 0.0)) {
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
        Context.packet.put("containsBlock", containsBlock)
        Context.packet.put("Intake Motor Current", intake.getCurrent(CurrentUnit.MILLIAMPS))
        Context.packet.put("Extension Real Position", extensionServos.realPosition)
        Context.packet.put("Drop Real Position", flip.realPosition)
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
        state = State.IN
    }

    private fun deploy() {
        Platform.isLoaded = false
        dropIntake()
        if (flip.isTransitioning) {
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

    override fun isTransitioningState(): Boolean = extensionServos.isTransitioning || flip.isTransitioning
}