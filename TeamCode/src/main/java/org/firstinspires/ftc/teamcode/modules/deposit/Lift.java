package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

public class Lift extends Module<Lift.State> {
    public enum State implements StateBuilder {
        MOVING,
        IDLE;

        @Override
        public double getTime() {
            return 0;
        }
    }
    public Lift(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
    }

    @Override
    public void init() {

    }

    @Override
    protected void internalUpdate() {

    }

    @Override
    protected boolean isDoingInternalWork() {
        return false;
    }

    @Override
    protected boolean isModuleInternalHazardous() {
        return false;
    }
}
