package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.deposit.Platform;
import org.firstinspires.ftc.teamcode.util.field.Details;

/**
 * Frontal mechanism for collecting freight
 *
 * @author Matthew Song
 */
public class Intake extends Module<Intake.State> {
    private DcMotorEx intake;
    private Servo outL, outR, flipL, flipR;
    private DistanceSensor blockSensor;
    private boolean isBlock;
    private double power;
    public enum State {
        PREP_OUT(0.3),
        TRANSIT_OUT(0.3),
        OUT(0),
        TRANSIT_IN(0.8),
        TRANSFER(0.2),
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
        outR = hardwareMap.get(Servo.class, "outR");
        outL = hardwareMap.get(Servo.class, "outL");
        flipR = hardwareMap.get(Servo.class, "flipL");
        flipL = hardwareMap.get(Servo.class, "flipR");
        blockSensor = hardwareMap.get(DistanceSensor.class, "block");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        slidesIn();
        flipL.setDirection(Servo.Direction.REVERSE);
        flipR.setDirection(Servo.Direction.REVERSE);
        raiseIntake();
    }

    public void setPower(double power) {
        if ((this.power > 0 && power <= 0) || (this.power < 0 && power >= 0) || (this.power == 0 && power != 0)) {
            if (power != 0 && !isDoingWork()) setState(State.PREP_OUT);
            else if (isDoingWork()) setState(State.TRANSIT_IN);
            Details.telemetry.addData("Transition Intake", getState());
        }
        this.power = power;
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */

    public boolean isDoingWork() {
        return getState() == State.TRANSIT_OUT || getState() == State.OUT || getState() == State.PREP_OUT;
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
                Details.telemetry.addData("Elapsed Time", elapsedTime.seconds());
            case IN:
                retract();
                break;
            case TRANSFER:
                power = -0.8;
                Platform.isLoaded = true;
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.IN);
                    power = 0;
                }
                break;
        }
        intake.setPower(power);
        Details.packet.put("Intake Velocity", intake.getVelocity());
        Details.telemetry.addData("Intake State", getState());
    }

    private void dropIntake() {
        flipR.setPosition(0.41);
        flipL.setPosition(0.59);
    }

    private void raiseIntake() {
        flipR.setPosition(0.9);
        flipL.setPosition(0.1);
    }

    private void deploy() {
        Platform.isLoaded = false;
        slidesOut();
        dropIntake();
    }

    private void retract() {
        slidesIn();
        raiseIntake();
    }

    private void slidesOut() {
        outL.setPosition(0.5);
        outR.setPosition(0.5);
    }

    private void slidesIn() {
        outL.setPosition(0);
        outR.setPosition(1);
    }
}
