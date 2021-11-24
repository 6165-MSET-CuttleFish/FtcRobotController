package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {
    DigitalChannel digitalChannel;

    public LimitSwitch(HardwareMap hardwareMap, String deviceName) {
        digitalChannel = hardwareMap.digitalChannel.get(deviceName);
    }

    public boolean isPressed() {
        return digitalChannel.getState();
    }
}
