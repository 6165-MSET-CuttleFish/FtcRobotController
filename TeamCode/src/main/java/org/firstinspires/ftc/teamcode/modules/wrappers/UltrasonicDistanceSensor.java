package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

import androidx.annotation.Nullable;

@Config
public class UltrasonicDistanceSensor extends Module<UltrasonicDistanceSensor.State> {
    //AnalogInput analogInput;
    DigitalChannel digitalInput;
    //public static double inchesPerVolt = 72.8834; //512
    public double distance;

    public enum State implements StateBuilder {
        SENDING,
        RECEIVING;

        @Nullable
        @Override
        public Double getTimeOut() {
            return null;
        }
    }

    @Override
    public boolean isTransitioningState() {
        return false;
    }

    //inches = 72.8834 * volts + 0.178

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String deviceName) {
        super(hardwareMap, State.SENDING);
        //analogInput = hardwareMap.analogInput.get(deviceName);
        digitalInput = hardwareMap.digitalChannel.get(deviceName);
    }
    @Override
    public void internalInit() {
    }


    protected void internalUpdate() {
        switch (getState()) {
            case SENDING:
                if (digitalInput.getState()) {
                    distance = getTimeSpentInState() * 330;
                    setState(State.RECEIVING);
                }
            case RECEIVING:
                setState(State.SENDING);
                break;
        }
    }

    @Override
    protected boolean isDoingInternalWork() {
        return false;
    }

    @Override
    protected boolean isModuleInternalHazardous() {
        return false;
    }

    public double getDistance() {

        return distance;
    }
}
