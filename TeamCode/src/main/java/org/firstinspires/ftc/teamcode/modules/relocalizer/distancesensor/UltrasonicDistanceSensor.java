package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class UltrasonicDistanceSensor {
    AnalogInput distanceSensor;
    Pose2d centerOffset;
    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String name, Pose2d centerOffset){
        distanceSensor = hardwareMap.analogInput.get(name);
        this.centerOffset = centerOffset;
    }
    public double getDistance(){
        return 89.4897 * distanceSensor.getVoltage() - 12.9012;
    }
}
