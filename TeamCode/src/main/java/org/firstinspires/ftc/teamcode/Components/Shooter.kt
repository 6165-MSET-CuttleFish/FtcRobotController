package org.firstinspires.ftc.teamcode.Components

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.util.InterpLUT
import com.noahbres.jotai.StateMachine
import com.noahbres.jotai.StateMachineBuilder
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Components.Turret
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.teamcode.util.TuningController
import org.firstinspires.ftc.teamcode.util.VelocityPIDFController

//http://192.168.43.1:8080/dash
@Config
class Shooter(hardwareMap: HardwareMap) : Component {
    enum class State {
        CONTINUOUS, CUSTOMVELO, POWERSHOTS, TUNING, IDLE, EMPTY_MAG
    }

    enum class PSState {
        MOVING_PS1, PS1, MOVING_PS2, PS2, MOVING_PS3, PS3
    }

    var state = State.IDLE
    @JvmField
    var powerShotsController: StateMachine<*>
    @JvmField
    var offset = 50.0
    var p = 0.0
    var a = 0.08
    private var lastTargetVelo = 0.0
    private var lastMotorPos = 0.0
    private var lastMotorVelo = 0.0
    private var lastAccel = 0.0
    private var lastKv = kV
    private var lastKa = kA
    private var lastKstatic = kStatic
    private var veloController = VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic)
    var tuner: TuningController
    var batteryVoltageSensor: VoltageSensor
    private val veloTimer = ElapsedTime()
    var flywheel: DcMotor
    var flywheel1: DcMotor
    var veloTracker: Encoder
    var flap: Servo
    var targetVelo = 0.0
    private val veloRegression: InterpLUT = InterpLUT()
    private val toleranceRegression: InterpLUT = InterpLUT()

    @JvmField
    var magazine: Magazine
    @JvmField
    var gunner: Gunner
    @JvmField
    var turret: Turret
    var timer = ElapsedTime()
    private val components: Array<Component>
    override fun update() {
        for (component in components) {
            component.update()
        }
        val shooterCoord = Coordinate.toPoint(Details.robotPose).polarAdd(Details.robotPose.heading - Math.PI, 3.0)
        when (state) {
            State.CONTINUOUS -> {
                flapUp()
                turret.target = Robot.goal()
                targetVelo = try {
                    veloRegression[shooterCoord.distanceTo(Coordinate.toPoint(Robot.goal()))] + offset
                } catch (e: Exception) {
                    4500.0
                }
            }
            State.POWERSHOTS -> {
                if (Details.opModeType == OpModeType.AUTO) {
                    targetVelo = 4100.0
                    flapUp()
                } else {
                    targetVelo = 4200.0
                    flapDown()
                }
                powerShotsController.update()
            }
            State.TUNING -> {
                if (!tuner.running) tuner.start()
                targetVelo = tuner.update()
            }
            State.IDLE -> {
                targetVelo = 0.0
                turret.state = Turret.State.IDLE
            }
            State.EMPTY_MAG -> {
                turret.setTargetAngle(Math.toRadians(-180.0))
                targetVelo = 2000.0
                if (turret.isIdle && Magazine.currentRings != 0.0 && turret.absoluteAngle < Math.toRadians(-100.0)) {
                    gunner.shoot()
                }
            }
            State.CUSTOMVELO -> {
                targetVelo = velocity
            }
        }
        veloController.setTargetVelocity(targetVelo)
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds())
        veloTimer.reset()
        lastTargetVelo = targetVelo
        val motorPos = veloTracker.currentPosition.toDouble()
        val motorVelo = lowPassFilter(veloTracker.correctedVelocity / 8192 * 60)
        val accel = motorVelo - lastMotorVelo
        val power: Double
        if (Math.abs(accel - lastAccel) < threshold) {
            power = veloController.update(motorPos, motorVelo)
            lastMotorVelo = motorVelo
            lastMotorPos = motorPos
            Details.packet.put("Shooter Velocity", motorVelo)
            Details.packet.put("Shooter Accel", accel)
        } else {
            power = veloController.update(lastMotorPos, lastMotorVelo)
            Details.packet.put("Shooter Velocity", lastMotorVelo)
            Details.packet.put("Shooter Accel", lastAccel)
        }
        lastAccel = accel
        if (targetVelo == 0.0) {
            flywheel.power = 0.0
            flywheel1.power = 0.0
        } else {
            flywheel.power = power
            flywheel1.power = power
        }
        if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
            lastKv = kV
            lastKa = kA
            lastKstatic = kStatic
            setPIDCoeffecients()
        }
        val newKv = kV * 12 / batteryVoltageSensor.voltage
        veloController.setkV(newKv)
        Details.packet.put("Target Velocity", targetVelo)
        Details.packet.put("Motor Power", power)
    }

    private fun setPIDCoeffecients() {
        veloController = VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic)
    }

    val shooterVec: Vector2d
        get() = Coordinate.toPoint(Details.robotPose).polarAdd(Details.robotPose.heading - Math.PI, 3.0).toVector()

    fun setVelocity(vector2d: Vector2d) {
        targetVelo = veloRegression[vector2d.distTo(Robot.goal())] + offset
    }

    fun getPoseVelo(vector2d: Vector2d): Double {
        return try {
            veloRegression[vector2d.distTo(Robot.goal())] + offset
        } catch (ignored: Exception) {
            4500.0
        }
    }

    var velocity: Double
        get() = veloTracker.correctedVelocity / 8192 * 60
        set(v) {
            state = State.CONTINUOUS
            targetVelo = v
        }
    private val error: Double
        get() = targetVelo - velocity
    private val absError: Double
        get() = Math.abs(error)
    val percentError: Double
        get() = absError / targetVelo

    fun flapUp() {
        flap.position = 0.85
    }

    fun flapDown() {
        flap.position = 0.87
    }

    fun flapWayDown() {
        flap.position = 1.0
    }

    private fun setVelocityController() {
        veloRegression.add(0.0, 4500.0)
        veloRegression.add(67.7, 4520.0)
        veloRegression.add(72.8, 4520.0)
        veloRegression.add(78.4, 4600.0)
        veloRegression.add(82.23, 4630.0)
        veloRegression.add(86.3, 4670.0)
        veloRegression.add(88.4, 4670.0)
        veloRegression.add(95.2, 4850.0)
        veloRegression.add(110.5, 5000.0)
        veloRegression.add(115.8, 5100.0)
        veloRegression.add(136.5, 5100.0)
        veloRegression.createLUT()
    }

    private fun setToleranceRegression() {
        toleranceRegression.add(0.0, 100.0)
        toleranceRegression.add(67.7, 100.0)
        toleranceRegression.add(72.8, 100.0)
        toleranceRegression.add(78.4, 100.0)
        toleranceRegression.add(82.23, 100.0)
        toleranceRegression.add(86.3, 80.0)
        toleranceRegression.add(95.2, 80.0)
        toleranceRegression.createLUT()
    }

    val isWithinTolerance: Boolean
        get() {
            val neededVelo = getPoseVelo(shooterVec)
            val error = Math.abs(velocity - neededVelo)
            val maxError: Double
            maxError = try {
                toleranceRegression[shooterVec.distTo(Robot.goal())]
            } catch (ignored: Exception) {
                80.0
            }
            return maxError > error
        }

    fun powerShots() {
        if (!powerShotsController.running) powerShotsController.start()
    }

    private fun lowPassFilter(i: Double): Double {
        val estimate = a * p + (1 - a) * i
        p = estimate
        return estimate
    }

    companion object {
        var MOTOR_VELO_PID = PIDCoefficients(0.001, 0.0, 0.0)
        var kV = 0.000154
        var kA = 0.000027
        var kStatic = 0.01
        var threshold = 350.0
    }

    init {
        setVelocityController()
        setToleranceRegression()
        flywheel = hardwareMap.get(DcMotor::class.java, "fw")
        flywheel1 = hardwareMap.get(DcMotor::class.java, "fw1")
        flywheel.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        flywheel.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheel.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        flywheel.direction = DcMotorSimple.Direction.REVERSE
        flywheel1.direction = DcMotorSimple.Direction.REVERSE
        flywheel.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        flywheel1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        veloTracker = Encoder(hardwareMap.get(DcMotorEx::class.java, "fw1"))
        veloTracker.direction = Encoder.Direction.REVERSE
        magazine = Magazine(hardwareMap)
        gunner = Gunner(hardwareMap)
        turret = Turret(hardwareMap)
        components = arrayOf(magazine, gunner, turret)
        flap = hardwareMap.get(Servo::class.java, "flap")
        if (Details.opModeType == OpModeType.AUTO) {
            flapWayDown()
        } else {
            flapDown()
        }
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        powerShotsController = StateMachineBuilder<PSState>()
                .state(PSState.MOVING_PS1)
                .transition { turret.isOnTarget && gunner.state == Gunner.State.IDLE || timer.seconds() > 0.6 }
                .onEnter {
                    timer.reset()
                    turret.target = Robot.powerShots()[0]
                }
                .state(PSState.PS1)
                .transitionTimed(4 * Gunner.gunTime)
                .onEnter { gunner.shoot() }
                .state(PSState.MOVING_PS2)
                .transition { turret.isOnTarget && gunner.state == Gunner.State.IDLE || timer.seconds() > 0.6 }
                .onEnter {
                    turret.target = Robot.powerShots()[1]
                    timer.reset()
                }
                .state(PSState.PS2)
                .transitionTimed(4 * Gunner.gunTime)
                .onEnter { gunner.shoot() }
                .state(PSState.MOVING_PS3)
                .transition { turret.isOnTarget && gunner.state == Gunner.State.IDLE || timer.seconds() > 0.6 }
                .onEnter {
                    timer.reset()
                    turret.target = Robot.powerShots()[2]
                }
                .state(PSState.PS3)
                .transitionTimed(4 * Gunner.gunTime)
                .onEnter { gunner.shoot() }
                .exit(PSState.MOVING_PS1)
                .onExit {
                    state = State.CONTINUOUS
                    turret.target = Robot.goal()
                }
                .build()
        tuner = TuningController()
        veloTimer.reset()
        setPIDCoeffecients()
    }
}