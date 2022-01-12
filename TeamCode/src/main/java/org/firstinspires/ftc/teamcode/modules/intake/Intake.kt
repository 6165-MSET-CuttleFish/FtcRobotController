package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modules.Module
import org.firstinspires.ftc.teamcode.modules.StateBuilder
import org.firstinspires.ftc.teamcode.modules.deposit.Platform
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.field.Context

/**
 * Frontal mechanism for collecting freight
 * @author Matthew Song
 */
@Config
class Intake(hardwareMap: HardwareMap) : Module<Intake.State>(hardwareMap, State.IN, Pose2d(7.7), 0.6) {
    companion object {
        @JvmField
        var raisedPosition = 0.32
        @JvmField
        var loweredPosition = 1.0
        @JvmField
        var distanceLimit = 18.0
        @JvmField
        var outPosition = 0.65
        @JvmField
        var inPosition = 0.88
    }
    enum class State(override val timeOut: Double, override val percentMotion: Double = 0.0) : StateBuilder {
        OUT(0.0, 1.0),
        TRANSFER(1.2),
        IN(0.0, 0.0),
        CREATE_CLEARANCE(0.3, 0.8),
    }

    private var intake = hardwareMap.get(DcMotorEx::class.java, "intake")
    private var outL = hardwareMap.servo["outL"]
    private var outR = hardwareMap.servo["outR"]
    private var outA = hardwareMap.servo["outA"]
    private var flip: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, "flipR")
    private var blockSensor = hardwareMap.get(ColorRangeSensor::class.java, "block")
    private var power = 0.0
    private val extendedTimer = ElapsedTime()
    private var containsBlock = false

    override fun internalInit() {
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        // extend range of servo by 30°
        flip.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        slidesIn()
        raiseIntake()
    }

    fun setPower(power: Double) {
        if (this.power > 0 && power <= 0 || this.power < 0 && power >= 0 || this.power == 0.0 && power != 0.0) {
            if (power != 0.0 && !isDoingInternalWork()) {
                state = State.OUT
                extendedTimer.reset()
            } else if (isDoingInternalWork()) {
                state = State.IN
            }
        }
        this.power = power
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    public override fun isDoingInternalWork(): Boolean {
        return state != State.IN && state != State.CREATE_CLEARANCE
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    public override fun isModuleInternalHazardous(): Boolean {
        return false
    }

    public override fun internalUpdate() {
        var power = power
        when (state) {
            State.OUT -> {
                deploy()
                extendedTimer.reset()
                if (distance < distanceLimit) {
                    state = State.IN
                    containsBlock = true
                }
            }
            State.IN -> {
                if (hasExceededTimeOut() && previousState == State.OUT) {
                    state = if (distance < distanceLimit) State.TRANSFER else State.IN
                }
                retract()
                extendedTimer.reset()
            }
            State.TRANSFER -> {
                power = -0.6
                Platform.isLoaded = true
                containsBlock = false
                if (distance > distanceLimit || timeSpentInState > state.timeOut) {
                    state = State.IN
                    power = 0.0
                    this.power = power
                }
            }
            State.CREATE_CLEARANCE -> {
                slidesOut()
                if (hasExceededTimeOut()) {
                    state = State.IN
                }
            }
        }
        poseOffset = Pose2d(7.7 + currentEstimatedPosition * 6.0)
        intake.power = power
        Context.packet.put("containsBlock", containsBlock)
        Context.packet.put("Intake Motor Current", intake.getCurrent(CurrentUnit.MILLIAMPS))
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

    private fun deploy() {
        Platform.isLoaded = false
        slidesOut()
        dropIntake()
    }

    private fun retract() {
        slidesIn()
        raiseIntake()
    }

    private fun slidesOut() {
        if (isHazardous) return
        outL.position = outPosition
        outR.position = outPosition
        outA.position = outPosition
    }

    private fun slidesIn() {
        outL.position = inPosition
        outR.position = inPosition
        outA.position = inPosition
    }
}