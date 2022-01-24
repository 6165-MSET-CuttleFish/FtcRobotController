package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import androidx.annotation.NonNull;

public abstract class ControllableServo extends ServoImplEx {
    ElapsedTime timer;
    double previousPosition;
    double positionMotionPerSecond = 1;
    boolean incrementingPosition = true;

    public ControllableServo(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
        super(controller, portNumber, servoType);
    }

    public double getRealPosition() {
        return incrementingPosition ? Range.clip(previousPosition + timer.seconds() * positionMotionPerSecond, previousPosition, getPosition())
        : Range.clip(previousPosition - timer.seconds() * positionMotionPerSecond, getPosition(), previousPosition);
    }

    @Override
    public void setPosition(double var1) {
        super.setPosition(var1);
        if (getPosition() == var1) return;
        incrementingPosition = getRealPosition() < var1;
        previousPosition = getRealPosition();
        timer.reset();
    }
}
