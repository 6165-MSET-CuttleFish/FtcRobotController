package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class UltrasonicDistanceSensor {
    AnalogInput analogInput;
    public static double inchesPerVolt = 72.8834; //512
    //inches = 72.8834 * volts + 0.178

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String deviceName) {
        analogInput = hardwareMap.analogInput.get(deviceName);
    }

    public double getDistance() {

        return analogInput.getVoltage() * inchesPerVolt + 0.178;
    }
}
