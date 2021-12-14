package org.firstinspires.ftc.teamcode.modules
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.field.Context

/**
 * This abstract class represents any module or subcomponent of the robot
 * NOTE: Do NOT use "sleeps" or any blocking clauses in any functions
 * @param <T> The state type of the module
 * @author Ayush Raman
</T> */
abstract class Module<T>(hardwareMap: HardwareMap, private var state: T) {

    /**
     * Instance of the hardwareMap passed to the constructor
     */
    @JvmField
    var hardwareMap: HardwareMap
    private var nestedModules = arrayOf<Module<*>>()

    /**
     * @return The previous state of the module
     */
    var previousState: T
        private set
    private val elapsedTime = ElapsedTime()

    /**
     * This function initializes all necessary hardware modules
     */
    abstract fun init()
    fun timeSpentInState(): Double {
        return elapsedTime.seconds()
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
        assert(Context.telemetry != null)
        Context.telemetry!!.addData(javaClass.simpleName + " State", getState())
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
     * @return The state of the module
     */
    fun getState(): T {
        return state
    }

    /**
     * Set a new state for the module
     * @param state New state of the module
     */
    protected open fun setState(state: T) {
        if (this.state === state) return
        elapsedTime.reset()
        previousState = this.state
        this.state = state
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
     * Note: Do NOT update any nested modules in this method, this will be taken care of automatically
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
     * Constructor which calls the 'init' function
     */
    init {
        previousState = state
        this.hardwareMap = hardwareMap
        init()
    }
}