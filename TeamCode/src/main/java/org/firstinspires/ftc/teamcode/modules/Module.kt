package org.firstinspires.ftc.teamcode.modules
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.roadrunnerext.polarAdd
import org.firstinspires.ftc.teamcode.util.field.Context
import org.firstinspires.ftc.teamcode.util.field.Context.robotPose

/**
 * This abstract class represents any module or subcomponent of the robot
 * NOTE: Do NOT use "sleeps" or any blocking clauses in any functions
 * @param <T> The state type of the module
 * @author Ayush Raman
</T> */
abstract class Module<T : StateBuilder> @JvmOverloads constructor(
    @JvmField var hardwareMap: HardwareMap,
    private var _state: T,
    var poseOffset: Pose2d = Pose2d()
) {
    private var nestedModules = arrayOf<Module<*>>()
    val modulePoseEstimate: Pose2d
        get() = robotPose.polarAdd(poseOffset.x).polarAdd(poseOffset.y, Math.PI / 2)

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
    private val elapsedTime = ElapsedTime()

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
     * The time spent in the current state in seconds
     */
    protected val timeSpentInState
        get() = elapsedTime.seconds()

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
        Context.telemetry?.addData(javaClass.simpleName + " State", state)
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
}