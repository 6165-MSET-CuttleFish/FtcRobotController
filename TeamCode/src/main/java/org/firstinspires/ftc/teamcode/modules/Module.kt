package org.firstinspires.ftc.teamcode.modules
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.util.Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.Actuator
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.roadrunnerext.toPose
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.robotPose

/**
 * This abstract class represents any module or subcomponent of the robot
 * NOTE: Do NOT use "sleeps" or any blocking clauses in any functions
 * @param <T> The state type of the module
 * @author Ayush Raman
</T> */
abstract class Module<T> @JvmOverloads constructor(
    @JvmField var hardwareMap: HardwareMap,
    private var _state: T,
    var poseOffset: Pose2d = Pose2d(),
) {
    /// State related fields
    private val elapsedTime = ElapsedTime()

    /**
     * @return The previous state of the module
     */
    var previousState: T = _state
        private set

    open var state: T
        /**
         * @return The state of the module
         */
        get() = _state
        /**
         * Set a new state for the module
         * @param value New state of the module
         */
        protected set(value) {
            if (state == value) return
            elapsedTime.reset()
            previousState = state
            _state = value
        }
    /**
     * The time spent in the current state in seconds
     */
    protected val secondsSpentInState
        get() = elapsedTime.seconds()

    /**
     * The time spent in the current state in milliseconds
     */
    protected val millisecondsSpentInState
        get() = elapsedTime.milliseconds()

    /// Module utilities
    private var nestedModules = arrayOf<Module<*>>()
    private var actuators = arrayOf<Actuator>()
    val modulePoseEstimate: Pose2d
        get() = robotPose.polarAdd(poseOffset.x).polarAdd(poseOffset.y, Math.PI / 2).vec().toPose(Angle.normDelta(poseOffset.heading + robotPose.heading))
    var isDebugMode = false
        set(value) {
            field = value
            nestedModules.forEach { it.isDebugMode = value }
        }

    /**
     * This function initializes all necessary hardware modules
     */
    abstract fun internalInit()

    fun init() {
        internalInit()
        nestedModules.forEach { it.init() }
    }

    /**
     * This function updates all necessary controls in a loop.
     * Prints the state of the module.
     * Updates all nested modules.
     */
    fun update() {
        nestedModules.forEach { it.update() }
        actuators.forEach { it.update() }
        internalUpdate()
        Context.packet.put(javaClass.simpleName + " State", if (isTransitioningState()) "$previousState --> $state" else state)
    }

    fun drawModule() {
        DashboardUtil.drawModule(Context.packet.fieldOverlay(), this)
    }

    /**
     * @return Whether the module or its nested modules are currently hazardous
     */
    var isHazardous: Boolean = false
        set(value) {
            field = value
            nestedModules.forEach { it.isHazardous = value }
            actuators.forEach { if (isHazardous) it.disable() else it.enable() }
        }

    /**
     * @return Whether the module or its nested modules are currently doing work
     */
    val isDoingWork: Boolean
        get() {
            var isDoingWork = isDoingInternalWork()
            nestedModules.forEach { if (it.isDoingWork) isDoingWork = true }
            return isDoingWork
        }

    /**
     * Add any nested modules to be updated
     */
    fun setNestedModules(vararg modules: Module<*>) {
        nestedModules = modules as Array<Module<*>>
    }

    /**
     * Add any nested modules to be updated
     */
    fun setActuators(vararg actuators: Actuator) {
        this.actuators = actuators as Array<Actuator>
    }

    /**
     * @return nested modules
     */
    fun getNestedModules(): Array<Module<*>> {
        return nestedModules
    }

    /**
     * This function updates all necessary controls in a loop
     * Note: Do NOT update any nested modules in this method. This will be taken care of automatically
     */
    protected abstract fun internalUpdate()

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary
     */
    protected abstract fun isDoingInternalWork(): Boolean

    /**
     * @return Whether the module is currently transitioning between states
     */
    abstract fun isTransitioningState(): Boolean
}