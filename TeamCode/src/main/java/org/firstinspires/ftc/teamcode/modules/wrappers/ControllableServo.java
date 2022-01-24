package org.firstinspires.ftc.teamcode.modules.wrappers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ControllableServo {
    ElapsedTime timer;
    Servo servo;
    double previousPosition;
    double totalMotionDuration = 1;
    boolean incrementingPosition = true;

    public ControllableServo(Servo servo) {
        this.servo = servo;
    }

    public double getRealPosition() {
        return incrementingPosition ? Range.clip(previousPosition + timer.seconds() / totalMotionDuration, previousPosition, servo.getPosition())
        : Range.clip(previousPosition - timer.seconds() / totalMotionDuration, servo.getPosition(), previousPosition);
    }

    public void setPosition(double var1) {
        servo.setPosition(var1);
        if (servo.getPosition() == var1) return;
        incrementingPosition = getRealPosition() < var1;
        previousPosition = getRealPosition();
        timer.reset();
    }

    public double getPosition() {
        return servo.getPosition();
    }
}
