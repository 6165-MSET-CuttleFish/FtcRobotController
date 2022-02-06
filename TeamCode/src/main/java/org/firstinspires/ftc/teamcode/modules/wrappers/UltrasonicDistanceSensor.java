package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

import androidx.annotation.Nullable;

@Config
public class UltrasonicDistanceSensor {
    AnalogInput distanceSensor;
    public UltrasonicDistanceSensor(HardwareMap map){
        distanceSensor = map.analogInput.get("ultra");
    }
    public double getDistance(){
        return 89.4897 * distanceSensor.getVoltage() - 12.9012;
    }
}
