package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class MB1643 implements DistanceSensor {
    AnalogInput distanceSensor;
    Pose2d centerOffset;

    public MB1643(HardwareMap hardwareMap, String deviceName) {
        distanceSensor = hardwareMap.analogInput.get(deviceName);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return 89.4897 * distanceSensor.getVoltage() - 12.9012;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
