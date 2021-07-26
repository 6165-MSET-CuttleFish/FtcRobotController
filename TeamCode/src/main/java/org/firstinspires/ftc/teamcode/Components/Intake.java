package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;

public class Intake implements Component {
    public enum IntakeState {
        SAFE,
        INTAKE,
        OUTTAKE,
        CUSTOM_VALUE,
        IDLE,
    }
    IntakeState intakeState = IntakeState.IDLE;
    DcMotor intakeMotor;
    Servo intakeL, intakeR;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL = hardwareMap.get(Servo.class, "intakeL");
        intakeR = hardwareMap.get(Servo.class, "intakeR");
    }

    @Override
    public void update() {
        if (opModeType == OpModeType.AUTO) {
            switch (intakeState) {

            }
        }
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setPower(double power) {
        setIntakeState(IntakeState.CUSTOM_VALUE);
        intakeMotor.setPower(power);
    }
}
