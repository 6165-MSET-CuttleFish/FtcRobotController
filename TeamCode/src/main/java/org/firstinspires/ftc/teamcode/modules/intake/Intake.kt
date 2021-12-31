package org.firstinspires.ftc.teamcode.modules.intake

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
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
class Intake(hardwareMap: HardwareMap) : Module<Intake.State>(hardwareMap, State.IN, Pose2d(7.7)) {
    companion object {
        @JvmStatic
        var raisedPosition = 0.19
        @JvmStatic
        var loweredPosition = 0.84
        @JvmStatic
        var distanceLimit = 18.0
        @JvmStatic
        var outPosition = 0.65
        @JvmStatic
        var inPosition = 0.19
    }
    enum class State(override val time: Double) : StateBuilder {
        PREP_OUT(0.3),
        TRANSIT_OUT(0.3),
        OUT(0.0),
        TRANSIT_IN(0.8),
        TRANSFER(0.7),
        IN(0.0);
    }

    private var intake = hardwareMap.get(DcMotorEx::class.java, "intake")
    private var outL = hardwareMap.servo["outL"]
    private var outR = hardwareMap.servo["outR"]
    private var outA = hardwareMap.servo["outA"]
    private var flipR = hardwareMap.servo["flipR"]
    private var blockSensor = hardwareMap.get(ColorRangeSensor::class.java, "block")
    private var power = 0.0
    private val extendedTimer = ElapsedTime()
    private var extendedDuration = 0.0

    override fun init() {
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")
        intake.mode = DcMotor.RunMode.RUN_USING_ENCODER
        intake.direction = DcMotorSimple.Direction.REVERSE
        outL = hardwareMap.servo["outL"]
        outR = hardwareMap.servo["outR"]
        outA = hardwareMap.servo["outA"]
        flipR = hardwareMap.servo["flipR"]
        slidesIn()
        raiseIntake()
    }

    fun setPower(power: Double) {
        if (this.power > 0 && power <= 0 || this.power < 0 && power >= 0 || this.power == 0.0 && power != 0.0) {
            if (power != 0.0 && !isDoingInternalWork()) {
                state = State.PREP_OUT
                extendedTimer.reset()
            } else if (isDoingInternalWork()) {
                state = State.TRANSIT_IN
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
                if (timeSpentInState > state.time) {
                    state = State.TRANSIT_OUT
                }
            }
            State.TRANSIT_OUT -> {
                if (timeSpentInState > state.time) {
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
                if (extendedDuration <= 0) {
                    state = if (0 < distanceLimit) State.TRANSFER else State.IN
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
                if (distance > distanceLimit || timeSpentInState > state.time) {
                    state = State.IN
                    power = 0.0
                    this.power = power
                }
            }
        }
        poseOffset = Pose2d(7.7 + extendedDuration * 6.0)
        intake.power = power
        Context.packet.put("Extended Duration", extendedDuration)
        Context.telemetry?.addData("Extended Duration", extendedDuration)
        val intakePose = Context.robotPose.polarAdd(7.7)
        DashboardUtil.drawIntake(Context.packet.fieldOverlay(), intakePose, modulePoseEstimate);
    }

    private val distance: Double
        get() = blockSensor.getDistance(DistanceUnit.CM)

    private fun dropIntake() {
        flipR.position = loweredPosition
    }

    private fun raiseIntake() {
        flipR.position = raisedPosition
    }

    private fun deploy() {
        Platform.isLoaded = false
        slidesOut();
        dropIntake()
    }

    private fun retract() {
        slidesIn()
        raiseIntake()
    }

    private fun slidesOut() {
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