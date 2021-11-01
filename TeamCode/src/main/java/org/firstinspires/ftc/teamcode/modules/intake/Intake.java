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
    public enum State {
        INTAKING,
        EXTAKING,
        IDLE
    }

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
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

        outL.setPosition(1);
        outR.setPosition(0.15);
        flipL.setDirection(Servo.Direction.REVERSE);
        flipR.setDirection(Servo.Direction.REVERSE);
        flipL.setPosition(.9);
        flipR.setPosition(0.1);

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
        switch(getState()){
            case INTAKING:
                in();
                break;
            case EXTAKING:
                out();
                break;
            case IDLE:
                off();
                break;
        }
        Details.packet.put("Intake Velocity", intake.getVelocity());
    }
    private void in(){
        intake.setPower(1);
        outL.setPosition(0.5);
        outR.setPosition(0.65);

        flipL.setPosition(0.43);
        flipR.setPosition(0.57);


        setState(State.INTAKING);


    }
    public int returnTicks(){
        return intake.getCurrentPosition();
    }
    private void out(){
        intake.setPower(-1);
        outL.setPosition(0.5);
        outR.setPosition(0.65);

        flipL.setPosition(0.43);
        flipR.setPosition(0.57);

        setState(State.EXTAKING);


    }
    private void off(){

        outL.setPosition(1);
        outR.setPosition(0.15);

        flipL.setPosition(0.9);
        flipR.setPosition(0.1);
        intakeDelay();

        setState(State.IDLE);

    }
    private void intakeDelay(){
        long timeElapsed = System.currentTimeMillis();
        int millis = 0;
        while(millis<300){
            millis += timeElapsed - System.currentTimeMillis();
        }
        intake.setPower(0);
    }


}
