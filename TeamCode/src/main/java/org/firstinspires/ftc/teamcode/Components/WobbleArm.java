package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class WobbleArm extends SubsystemBase {
    State state = State.IN;

    public State getState() {
        return state;
    }

    enum State{
        OUT,
        IN,
        MID,
    }

}
