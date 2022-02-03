package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;

import androidx.annotation.Nullable;

@Config
public class UltrasonicDistanceSensor extends Module<UltrasonicDistanceSensor.State> {
    DigitalChannel trig;
    DigitalChannel echo;
    private double distance;
    public static double halfSpeedOfSound = 13503.9 / 2.0;

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

    public UltrasonicDistanceSensor(HardwareMap hardwareMap, String outName, String inName) {
        super(hardwareMap, State.SENDING_LOW);
        trig = hardwareMap.digitalChannel.get(outName);
        echo = hardwareMap.digitalChannel.get(inName);
    }

    @Override
    public void internalInit() {
        trig.setMode(DigitalChannel.Mode.OUTPUT);
        echo.setMode(DigitalChannel.Mode.INPUT);
    }

    protected void internalUpdate() {
        trig.setState(false);
        switch (getState()) {
            case SENDING_LOW:
                //trig.setMode(DigitalChannel.Mode.OUTPUT);
                trig.setState(false);
                if (getMicrosecondsSpentInState() > 2) {
                    setState(State.SENDING_HIGH);
                }
                break;
            case SENDING_HIGH:
                //trig.setMode(DigitalChannel.Mode.OUTPUT);
                trig.setState(true);
                if (getMicrosecondsSpentInState() > 10) {
                    trig.setState(false);
                    setState(State.RECEIVING);
                }
                break;
            case RECEIVING:
                // echo.setMode(DigitalChannel.Mode.INPUT);
                if (echo.getState()) {
                    distance = getSecondsSpentInState() * halfSpeedOfSound;
                    setState(State.SENDING_LOW);
                } else {
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

    public double getMicros() {
        return getMicrosecondsSpentInState();
    }
}
