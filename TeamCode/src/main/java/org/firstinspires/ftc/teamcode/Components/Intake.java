package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;

public class Intake implements Component {
    public enum State {
        UP,
        DOWN,
        MID,
    }
    public static double TICKS_PER_REV;
    public static double GEAR_RATIO;
    State state = State.UP;
    DcMotor intakeMotor;
    Servo intakeL, intakeR;

    public Intake(HardwareMap hardwareMap) {
        switch (opModeType) {
            case AUTO:
                state = State.UP;
                break;
            case TELE:
                state = State.DOWN;
                break;
        }
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL = hardwareMap.get(Servo.class, "intakeL");
        intakeR = hardwareMap.get(Servo.class, "intakeR");
    }

    @Override
    public void update() {
        switch (state) {
            case UP:
                shieldUp();
                intakeMotor.setTargetPosition((int) getClosestZero());
                if(intakeMotor.getTargetPosition() > intakeMotor.getCurrentPosition())
                    intakeMotor.setPower(1);
                else
                    intakeMotor.setPower(-1);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                break;
            case DOWN:
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shieldDown();
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    private double getRotationCount() {
        return TICKS_PER_REV * GEAR_RATIO;
    }

    private double getClosestZero() {
        double delta = intakeMotor.getCurrentPosition() % getRotationCount();
        if (delta > getRotationCount()/2) delta = getRotationCount() - delta;
        double target1 = intakeMotor.getCurrentPosition();
        while (target1 % getRotationCount() > 1) {
            target1 ++;
        }
        double target2 = intakeMotor.getCurrentPosition();
        while (target2 % getRotationCount() > 1) {
            target2 --;
        }
        double range1 = Math.abs(getRotationCount() - target1);
        double range2 = Math.abs(getRotationCount() - target2);
        return range1 < range2 ? target1 : target2;
    }

    private void shieldUp() {
        intakeL.setPosition(0.16);
        intakeR.setPosition(1);
    }

    private void shieldMid() {
        intakeL.setPosition(0.16);
        intakeR.setPosition(0.95);
    }

    private void shieldDown() {
        intakeL.setPosition(0.28);
        intakeR.setPosition(0.88);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}