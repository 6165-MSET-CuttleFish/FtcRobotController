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
    IntakeState intakeState = IntakeState.CUSTOM_VALUE;
    DcMotor intakeMotor;
    CRServo intakeL, intakeR;
    Servo drop;


    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL = hardwareMap.crservo.get("intakeL");
        intakeR = hardwareMap.crservo.get("intakeR");
        drop = hardwareMap.servo.get("drop");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void update() {
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

    //use these two functions so that it drops intake and then raises the dropper
    public void dropIntake(){ drop.setPosition(0.6);}
    public void raiseIntake(){drop.setPosition(1);}

    //public double getDropPosition(){return drop.getPosition();}
    public void setPower(double power) {
        setIntakeState(IntakeState.CUSTOM_VALUE);
        intakeMotor.setPower(power);
        setVectors(power);
    }
}
