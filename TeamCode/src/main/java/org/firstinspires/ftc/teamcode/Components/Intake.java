package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;

public class Intake implements Component {
    public enum IntakeState {
        SAFE,
        CUSTOM_VALUE,
        IDLE,
    }
    IntakeState intakeState = IntakeState.IDLE;
    DcMotor intakeMotor;
    CRServo intakeL, intakeR;
    Servo drop;
    ColorRangeSensor colorRangeSensor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL = hardwareMap.crservo.get("intakeL");
        intakeR = hardwareMap.crservo.get("intakeR");
        drop = hardwareMap.servo.get("drop");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "range");
    }

    @Override
    public void update() {
        switch (intakeState) {
            case IDLE:
                setPower(0);
                break;
            case SAFE:
                if (colorRangeSensor.getDistance(DistanceUnit.INCH) < 1) {
                    setIntakeState(IntakeState.IDLE);
                    break;
                }
                setPower(1);
                break;
        }
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    private void setVectors(double power) {
        intakeL.setPower(power);
        intakeR.setPower(power);
    }
    public void dropIntake(){
        drop.setPosition(1/*insert positions */);
    }
    public void setPower(double power) {
        setIntakeState(IntakeState.CUSTOM_VALUE);
        intakeMotor.setPower(power);
        setVectors(power);
    }
}
