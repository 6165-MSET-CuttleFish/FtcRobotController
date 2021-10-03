package org.firstinspires.ftc.teamcode.modules.weigher;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Weighing the Freight to determine type
 *
 * @author Matthew Song
 */
public class Weigher extends Module<Weigher.State> {
    enum State {
        NONE,
        BALL,
        CUBE_LIGHT,
        CUBE_MEDIUM,
        CUBE_HEAVY
    }
    State state = State.NONE;

    /**
     * Constructor which calls the 'init' function
     *
     * @param hardwareMap instance of the hardware map provided by the OpMode
     */
    public Weigher(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void update() {

    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void init() {

    }

    /**
     * @return The state of the module
     */
    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean isHazardous() {
        return false;
    }
}
