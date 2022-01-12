package org.firstinspires.ftc.teamcode.modules
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.robotPose
import kotlin.math.abs

/**
 * This abstract class represents any module or subcomponent of the robot
 * NOTE: Do NOT use "sleeps" or any blocking clauses in any functions
 * @param <T> The state type of the module
 * @author Ayush Raman
</T> */
abstract class Module<T : StateBuilder> @JvmOverloads constructor(
    @JvmField var hardwareMap: HardwareMap,
    private var _state: T,
    var poseOffset: Pose2d = Pose2d(),
    var totalMotionDuration: Double = 1.0
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
            incrementPosition = value.percentMotion > state.percentMotion
            previousEstimatedPosition = currentEstimatedPosition
            elapsedTime.reset()
            previousState = state
            _state = value
        }
    /**
     * The time spent in the current state in seconds
     */
    protected val timeSpentInState
        get() = elapsedTime.seconds()

    private val timeOut: Double
        get() = abs(previousState.percentMotion - state.percentMotion) * totalMotionDuration
    protected fun hasExceededTimeOut(): Boolean = timeSpentInState > timeOut

    /// Module utilities
    private var nestedModules = arrayOf<Module<*>>()
    val modulePoseEstimate: Pose2d
        get() = robotPose.polarAdd(poseOffset.x).polarAdd(poseOffset.y, Math.PI / 2)
    var isDebugMode = false
        set(value) {
            field = value
            for (module in nestedModules) {
                module.isDebugMode = value
            }
        }

    /// Position estimating
    /**
     * Whether the current state is above or below the previous state
     */
    private var incrementPosition = true
    /**
     * @return previous position as a percentage of the [totalMotionDuration]
     */
    var previousEstimatedPosition = 0.0
        private set
    /**
     * @return current position as a percentage of the [totalMotionDuration]
     */
    var currentEstimatedPosition: Double = 0.0
        private set
        get() =
            if (incrementPosition) Range.clip(previousEstimatedPosition + timeSpentInState / totalMotionDuration, previousState.percentMotion, state.percentMotion)
            else Range.clip(previousEstimatedPosition - timeSpentInState / totalMotionDuration, state.percentMotion, previousState.percentMotion)

    /**
     * This function initializes all necessary hardware modules
     */
    abstract fun internalInit()

    fun init() {
        internalInit()
        for (module in nestedModules) {
            module.init()
        }
    }

    /**
     * This function updates all necessary controls in a loop.
     * Prints the state of the module.
     * Updates all nested modules.
     */
    fun update() {
        for (module in nestedModules) {
            module.update()
        }
        internalUpdate()
        Context.packet.put(javaClass.simpleName + " State", if (isTransitioningState()) "$previousState --> $state" else state)
        Context.packet.put(javaClass.simpleName + " Position", currentEstimatedPosition)
        Context.packet.put(javaClass.simpleName + " is doing work?", isDoingWork)
    }

    /**
     * @return Whether the module or its nested modules are currently hazardous
     */
    val isHazardous: Boolean
        get() {
            var isHazardous = isModuleInternalHazardous()
            for (module in nestedModules) {
                if (module.isHazardous) {
                    isHazardous = true
                }
            }
            return isHazardous
        }

    /**
     * @return Whether the module or its nested modules are currently doing work
     */
    val isDoingWork: Boolean
        get() {
            var isDoingWork = isDoingInternalWork()
            for (module in nestedModules) {
                if (module.isDoingWork) {
                    isDoingWork = true
                }
            }
            return isDoingWork
        }

    /**
     * Add any nested modules to be updated
     */
    fun setNestedModules(vararg modules: Module<*>) {
        nestedModules = modules as Array<Module<*>>
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
     * @return Whether the module is currently in a hazardous state
     */
    protected abstract fun isModuleInternalHazardous(): Boolean

    /**
     * @return Whether the module is currently transitioning between states
     */
     open fun isTransitioningState(): Boolean = !hasExceededTimeOut()
}