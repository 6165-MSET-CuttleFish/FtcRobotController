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
    enum State {
        INTAKING,
        EXTAKING,
        IDLE
    }

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outL = hardwareMap.get(Servo.class, "outL");
        outR = hardwareMap.get(Servo.class, "outR");
        flipL = hardwareMap.get(Servo.class, "flipL");
        flipR = hardwareMap.get(Servo.class, "flipR");
        //blockSensor = hardwareMap.get(DistanceSensor.class, "block");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outL.setPosition(1);
        outR.setPosition(0.15);
        flipL.setPosition(1);
        flipR.setPosition(0.9);
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
            case EXTAKING:
                out();
            case IDLE:
                off();
        }
        Details.packet.put("ticks", intake.getVelocity());
    }
    private void in(){
        intake.setPower(1);
        outL.setPosition(0.5);
        outR.setPosition(0.65);
        flipL.setPosition(0.57);
        flipR.setPosition(0.43);
        setState(State.INTAKING);

    }
    public int returnTicks(){
        return intake.getCurrentPosition();
    }
    private void out(){

        intake.setPower(-1);
        outL.setPosition(0.5);
        outR.setPosition(0.65);
        flipL.setPosition(0.57);
        flipR.setPosition(0.43);
        setState(State.EXTAKING);

    }
    private void off(){
        intake.setPower(0);
        outL.setPosition(1);
        outR.setPosition(0.15);
        flipL.setPosition(1);
        flipR.setPosition(0.9);
        setState(State.IDLE);

    }


}
