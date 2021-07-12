package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;

public class Intake implements Component {
    public enum State {
        UP,
        DOWN
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
        if (opModeType == OpModeType.AUTO) intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL = hardwareMap.get(Servo.class, "intakeL");
        intakeR = hardwareMap.get(Servo.class, "intakeR");
    }

    @Override
    public void update() {
        switch (state) {
            case UP:
                shieldUp();
                break;
            case DOWN:
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

    public void shieldUp() {
        state = State.UP;
        intakeL.setPosition(0);
        intakeR.setPosition(1);
    }

    public void shieldDown() {
        state = State.DOWN;
        intakeL.setPosition(0.12);
        intakeR.setPosition(0.88);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}
