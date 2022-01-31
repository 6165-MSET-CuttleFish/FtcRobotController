package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

import androidx.annotation.Nullable;

@Config
public class UltrasonicDistanceSensor extends Module<UltrasonicDistanceSensor.State> {
    DigitalChannel digitalChannel;
    private double distance;
    public static double constant = 7.40525e-5 / 2.0;

    public enum State implements StateBuilder {
        SENDING_LOW,
        SENDING_HIGH,
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

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String deviceName) {
        super(hardwareMap, State.SENDING_LOW);
        digitalChannel = hardwareMap.digitalChannel.get(deviceName);
    }
    @Override
    public void internalInit() {
        digitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
        digitalChannel.setState(false);
    }

    protected void internalUpdate() {
        switch (getState()) {
            case SENDING_LOW:
                digitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
                digitalChannel.setState(false);
                if (getMicrosecondsSpentInState() > 2) {
                    setState(State.SENDING_HIGH);
                }
                break;
            case SENDING_HIGH:
                digitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
                digitalChannel.setState(true);
                if (getMicrosecondsSpentInState() > 10) {
                    setState(State.RECEIVING);
                }
                break;
            case RECEIVING:
                digitalChannel.setMode(DigitalChannel.Mode.INPUT);
                if (digitalChannel.getState()) {
                    distance = getMillisecondsSpentInState() * constant;
                    setState(State.SENDING_LOW);
                }
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

    /**
     *
     * @return distance in inches
     */
    public double getDistance() {
        return distance;
    }
}
