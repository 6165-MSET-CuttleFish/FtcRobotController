package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.Details;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
        TRANSIT_OUT(0.3),
        OUT(0),
        TRANSIT_IN(0.3),
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
        outL = hardwareMap.get(Servo.class, "outR");
        outR = hardwareMap.get(Servo.class, "outL");
        flipL = hardwareMap.get(Servo.class, "flipL");
        flipR = hardwareMap.get(Servo.class, "flipR");
        //blockSensor = hardwareMap.get(DistanceSensor.class, "block");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
//        outL.setPosition(1);
//        outR.setPosition(0.15);
        flipL.setDirection(Servo.Direction.REVERSE);
        flipR.setDirection(Servo.Direction.REVERSE);
//        flipL.setPosition(.9);
//        flipR.setPosition(0.1);

    }



    public void setPower(double power) {
        this.power = power;
        if (power != 0) setState(State.TRANSIT_OUT);
        else setState(State.TRANSIT_IN);
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */

    public boolean isDoingWork() {
        return false;
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
        intake.setPower(power);
        switch(getState()){
            case TRANSIT_OUT:
                if (elapsedTime.seconds() > getState().time) {
                    setState(State.OUT);
                }
            case OUT:
                in();
                break;
            case TRANSIT_IN:
                if(elapsedTime.seconds() > getState().time && elapsedTime.seconds() < getState().time + 0.2) {
                    intake.setPower(-0.8);
                    setState(State.IN);
                }
            case IN:
                in();
                break;
        }
        Details.packet.put("Intake Velocity", intake.getVelocity());
    }
    private void out(){
        intake.setPower(1);
        outL.setPosition(0.5);
        outR.setPosition(0.65);

        flipL.setPosition(0.43);
        flipR.setPosition(0.57);


    }

    private void in(){
        intake.setPower(0);
        outL.setPosition(1);
        outR.setPosition(0.15);

        flipL.setPosition(0.9);
        flipR.setPosition(0.1);

    }
    private void motorOff(){
        intake.setPower(0);
    }


}
