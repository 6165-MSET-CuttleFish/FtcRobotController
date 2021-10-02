package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This abstract class represents any module or subcomponent of the robot
 * NOTE: Do NOT use "sleeps" or any blocking clauses in any functions
 *
 * @author Ayush Raman
 */
public abstract class Module {
    public HardwareMap hardwareMap;

    /**
     * Constructor which calls the 'init' function
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Module(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    public abstract void update();

    /**
     * This function initializes all necessary hardware modules
     */
    public abstract void init();
}
