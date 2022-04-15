package org.firstinspires.ftc.teamcode.modules.relocalizer.distancesensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class MB1643 implements DistanceSensor {
    AnalogInput distanceSensor;

    public MB1643(HardwareMap hardwareMap, String deviceName) {
        distanceSensor = hardwareMap.analogInput.get(deviceName);
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        double v;
        try {
            v = distanceSensor.getVoltage();
        } catch (Exception e) {
            v = 0;
        }
        double inches = 89.4897 * v - 12.9012 + .625;
        switch (unit) {
            case INCH: return inches;
            case CM: return inches * 2.54;
            case MM: return inches * 25.4;
            case METER: return inches * 0.0254;
        }
        return inches;
    }

    @Override
    public Manufacturer getManufacturer() {
        return distanceSensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return distanceSensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return distanceSensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return distanceSensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        distanceSensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        distanceSensor.close();
    }
}
