package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.deposit.Platform;
import org.firstinspires.ftc.teamcode.util.field.Details;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

/**
 * Frontal mechanism for collecting freight
 *
 * @author Matthew Song
 */
public class Intake extends Module<Intake.State> {
    private DcMotorEx intake;
    private Servo outL, outR, flipL, flipR;
    private ColorRangeSensor blockSensor;
    private double power;
    public enum State {
        PREP_OUT(0.3),
        TRANSIT_OUT(0.3),
        OUT(0),
        TRANSIT_IN(0.5),
        TRANSFER(0.7),
        IN(0);
        final double time;
        State(double time){
            this.time = time;
        }
    }

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, State.IN);
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outR = hardwareMap.servo.get("outR");
        outL = hardwareMap.servo.get("outL");
        outR.setDirection(Servo.Direction.REVERSE);
        flipR = hardwareMap.servo.get("flipR");
        flipL = hardwareMap.servo.get("flipL");
        blockSensor = hardwareMap.get(ColorRangeSensor.class, "block");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        slidesIn();
        raiseIntake();
    }

    public void setPower(double power) {
        if ((this.power > 0 && power <= 0) || (this.power < 0 && power >= 0) || (this.power == 0 && power != 0)) {
            if (power != 0 && !isDoingWork()) {
                if (getState() == State.IN) setState(State.PREP_OUT);
                else setState(State.PREP_OUT);
            }
            else if (isDoingWork()) {
                setState(State.TRANSIT_IN);
            }
        }
        this.power = power;
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */

    public boolean isDoingWork() {
        return getState() != State.IN && getState() != State.TRANSIT_IN;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

    @Override
    public void update() {
        double power = this.power;
        switch(getState()){
            case PREP_OUT:
                dropIntake();
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.TRANSIT_OUT);
                }
                break;
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                deploy();
                break;
            case TRANSIT_IN:
                if(elapsedTime.seconds() > getState().time) {
                    if (blockSensor.getDistance(DistanceUnit.CM) < 10) {
                        setState(State.TRANSFER);
                    } else {
                        setState(State.IN);
                    }
                }
                power = 0.8;
            case IN:
                retract();
                break;
            case TRANSFER:
                power = -1;
                Platform.isLoaded = true;
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.IN);
                    this.power = power = 0;
                }
                break;
        }
        intake.setPower(power);
    }

    private void dropIntake() {
        flipR.setPosition(0.40);
        flipL.setPosition(0.60);
    }

    private void raiseIntake() {
        flipR.setPosition(0.87);
        flipL.setPosition(0.13);
    }

    private void deploy() {
        Platform.isLoaded = false;
        // slidesOut();
        dropIntake();
    }

    private void retract() {
        slidesIn();
        raiseIntake();
    }

    private void slidesOut() {
        if (Details.opModeType == OpModeType.AUTO) return;
        outL.setPosition(0.65);
        outR.setPosition(0.65);
    }

    private void slidesIn() {
        outL.setPosition(0.19);
        outR.setPosition(0.19);
    }
}
