package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
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
class Intake(hardwareMap: HardwareMap) : Module<Intake.State>(hardwareMap, State.IN) {
    companion object {
        @JvmStatic
        var raisedPosition = 0.88
        @JvmStatic
        var loweredPosition = 0.4
        @JvmStatic
        var distanceLimit = 18.0
    }
    enum class State(override val time: Double, override val isTransitionState: Boolean = false, override val nextState: State? = null) : StateBuilder {
        PREP_OUT(0.3),
        TRANSIT_OUT(0.3,true),
        OUT(0.0),
        TRANSIT_IN(0.5,true),
        TRANSFER(1.5),
        IN(0.0);
    }

    private val intake = hardwareMap.get(DcMotorEx::class.java, "intake")
    private val outL = hardwareMap.servo["outL"]
    private val outR = hardwareMap.servo["outR"]
    private val flipL = hardwareMap.servo["flipL"]
    private val flipR = hardwareMap.servo["flipR"]
    private var blockSensor = hardwareMap.get(ColorRangeSensor::class.java, "block")
    private var power = 0.0
    private val extendedTimer = ElapsedTime()
    private var extendedDuration = 0.0

    override fun init() {
        outR.direction = Servo.Direction.REVERSE
        intake.mode = DcMotor.RunMode.RUN_USING_ENCODER
        intake.direction = DcMotorSimple.Direction.REVERSE
        // slidesIn()
        raiseIntake()
    }

    fun setPower(power: Double) {
        if (this.power > 0 && power <= 0 || this.power < 0 && power >= 0 || this.power == 0.0 && power != 0.0) {
            if (power != 0.0 && !isDoingInternalWork()) {
                state = State.PREP_OUT
                extendedTimer.reset()
            } else if (isDoingInternalWork()) {
                state = State.TRANSIT_IN
                extendedDuration = extendedTimer.seconds()
            }
        }
        this.power = power
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    public override fun isDoingInternalWork(): Boolean {
        return state != State.IN && state != State.TRANSIT_IN
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
            State.PREP_OUT -> {
                dropIntake()
                if (timeSpentInState() > state.time) {
                    state = State.TRANSIT_OUT
                }
            }
            State.TRANSIT_OUT -> {
                if (timeSpentInState() > state.time) {
                    state = State.OUT
                }
                if (state != State.PREP_OUT) deploy()
                extendedDuration = Range.clip(
                    extendedDuration + extendedTimer.seconds(),
                    0.0,
                    State.TRANSIT_IN.time
                )
                extendedTimer.reset()
            }
            State.OUT -> {
                if (state != State.PREP_OUT) deploy()
                extendedDuration = Range.clip(
                    extendedDuration + extendedTimer.seconds(),
                    0.0,
                    State.TRANSIT_IN.time
                )
                extendedTimer.reset()
            }
            State.TRANSIT_IN -> {
                if (timeSpentInState() > extendedDuration) {
                    state = if (distance < distanceLimit) State.TRANSFER else State.IN
                }
                power = 0.8
                retract()
                extendedDuration = Range.clip(
                    extendedDuration - extendedTimer.seconds(),
                    0.0,
                    State.TRANSIT_IN.time
                )
                extendedTimer.reset()
            }
            State.IN -> {
                retract()
                extendedDuration = Range.clip(
                    extendedDuration - extendedTimer.seconds(),
                    0.0,
                    State.TRANSIT_IN.time
                )
                extendedTimer.reset()
            }
            State.TRANSFER -> {
                power = -1.0
                Platform.isLoaded = true
                if (distance > distanceLimit || timeSpentInState() > state.time) {
                    state = State.IN
                    power = 0.0
                    this.power = power
                }
            }
        }
        intake.power = power
        Context.packet.put("Extended Duration", extendedDuration)
        val intakePose = Context.robotPose.polarAdd(7.7)
        DashboardUtil.drawIntake(Context.packet.fieldOverlay(), intakePose, intakePose.polarAdd(extendedDuration * 6.0));
    }

    private val distance: Double
        get() = blockSensor.getDistance(DistanceUnit.CM)

    private fun dropIntake() {
        flipR.position = loweredPosition
        flipL.position = 1 - loweredPosition
    }

    private fun raiseIntake() {
        flipR.position = raisedPosition
        flipL.position = 1 - raisedPosition
    }

    private fun deploy() {
        Platform.isLoaded = false
        // slidesOut();
        dropIntake()
    }

    private fun retract() {
        // slidesIn()
        raiseIntake()
    }

    private fun slidesOut() {
        outL.position = 0.65
        outR.position = 0.65
    }

    private fun slidesIn() {
        outL.position = 0.19
        outR.position = 0.19
    }
}