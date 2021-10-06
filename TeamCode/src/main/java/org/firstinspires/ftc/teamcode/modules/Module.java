package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This abstract class represents any module or subcomponent of the robot
 * NOTE: Do NOT use "sleeps" or any blocking clauses in any functions
 * @param <T> The state type of the module
 * @author Ayush Raman
 */
public abstract class Module<T> {
    /**
     * Instance of the hardwareMap passed to the constructor
     */
    public HardwareMap hardwareMap;
    private T state;

    /**
     * Constructor which calls the 'init' function
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Module(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    /**
     * This function initializes all necessary hardware modules
     */
    public abstract void init();

    /**
     * This function updates all necessary controls in a loop
     */
    public abstract void update();

    /**
     * @return The state of the module
     */
    public T getState() {
        return state;
    }

    /**
     * Set a new state for the module
     * @param state New state of the module
     */
    public void setState(T state) {
        this.state = state;
    }

    /**
     *
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    public abstract boolean isHazardous();
}
