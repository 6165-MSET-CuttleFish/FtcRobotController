package org.firstinspires.ftc.teamcode.modules.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.deposit.Platform;
import org.firstinspires.ftc.teamcode.util.field.Context;

/**
 * Frontal mechanism for collecting freight
 * @author Matthew Song
 */
@Config
public class Intake extends Module<Intake.State> {
    public static double raisedPosition = 0.88;
    public static double loweredPosition = 0.4;

    public enum State {
        PREP_OUT(0.3),
        TRANSIT_OUT(0.3),
        OUT(0),
        TRANSIT_IN(0.5),
        TRANSFER(1.5),
        IN(0);
        final double time;
        State(double time){
            this.time = time;
        }
    }

    private DcMotorEx intake;
    public static double distanceLimit = 18;
    private Servo outL, outR, flipL, flipR;
    private ColorRangeSensor blockSensor;
    private double power;
    private final ElapsedTime extendedTimer = new ElapsedTime();
    private double extendedDuration, retractedDuration;

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
            if (power != 0 && !isDoingInternalWork()) {
                setState(State.PREP_OUT);
                extendedTimer.reset();
            } else if (isDoingInternalWork()) {
                setState(State.TRANSIT_IN);
                extendedDuration = extendedTimer.seconds();
            }
        }
        this.power = power;
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */

    public boolean isDoingInternalWork() {
        return getState() != State.IN && getState() != State.TRANSIT_IN;
    }

    /**
     * @return Whether the module is currently in a hazardous state
     */
    @Override
    public boolean isModuleInternalHazardous() {
        return false;
    }

    @Override
    public void internalUpdate() {
        double power = this.power;
        switch(getState()){
            case PREP_OUT:
                dropIntake();
                if (timeSpentInState() > getState().time) {
                    setState(State.TRANSIT_OUT);
                }
                break;
            case TRANSIT_OUT:
                if (timeSpentInState() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                if (getState() != State.PREP_OUT) deploy();
                extendedDuration = Range.clip(extendedDuration + extendedTimer.seconds(), 0, State.TRANSIT_IN.time);
                extendedTimer.reset();
                break;
            case TRANSIT_IN:
                if(timeSpentInState() > extendedDuration) {
                    setState(getDistance() < distanceLimit ? State.TRANSFER : State.IN);
                }
                power = 0.8;
            case IN:
                retract();
                extendedDuration = Range.clip(extendedDuration - extendedTimer.seconds(), 0, State.TRANSIT_IN.time);
                extendedTimer.reset();
                break;
            case TRANSFER:
                power = -1;
                Platform.isLoaded = true;
                if (getDistance() > distanceLimit || timeSpentInState() > getState().time) {
                    setState(State.IN);
                    this.power = power = 0;
                }
                break;
        }
        intake.setPower(power);
        assert Context.telemetry != null;
        Context.telemetry.addData("Extended Duration", extendedDuration);
    }

    private double getDistance() {
        return blockSensor.getDistance(DistanceUnit.CM);
    }

    private void dropIntake() {
        flipR.setPosition(loweredPosition);
        flipL.setPosition(1 - loweredPosition);
    }

    private void raiseIntake() {
        flipR.setPosition(raisedPosition);
        flipL.setPosition(1 - raisedPosition);
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
        outL.setPosition(0.65);
        outR.setPosition(0.65);
    }

    private void slidesIn() {
//        outL.setPosition(0.19);
//        outR.setPosition(0.19);
    }
}
