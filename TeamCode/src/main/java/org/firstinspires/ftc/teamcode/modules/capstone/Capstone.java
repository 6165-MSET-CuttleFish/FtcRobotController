package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;

// TODO: Wrapper class for slides and arm
public class Capstone extends Module<Capstone.State> {

    public enum State {
        IDLE,
        PICKUP,
        DROP,
    }

    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isDoingWork() {
        return false;
    }

    @Override
    public boolean isHazardous() {
        return false;
    }
}
