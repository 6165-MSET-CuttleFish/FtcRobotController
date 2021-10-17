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
    private Servo dropL, dropR;
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
        dropL = hardwareMap.get(Servo.class, "dropL");
        dropR = hardwareMap.get(Servo.class, "dropR");
        blockSensor = hardwareMap.get(DistanceSensor.class, "block");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        isBlock = false;
        dropL.setPosition(0.25);
        dropR.setDirection(Servo.Direction.REVERSE);
        dropR.setPosition(0.95);
    }

    @Override
    public State getState() {

        return this.getState();
    }

    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */

    public boolean isDoingWork() {
        return false;
    }


    public void update() {
        functions();
        Details.packet.put("ticks", intake.getVelocity());
    }
    private void in(){
        intake.setPower(1);
        setState(State.INTAKING);
        dropL.setPosition(0.45);
        dropR.setPosition(0.75);
    }
    public int returnTicks(){
        return intake.getCurrentPosition();
    }
    private void out(){
        intake.setPower(-1);
        setState(State.EXTAKING);
        dropL.setPosition(0.55);
        dropR.setPosition(0.85);
    }
    private void off(){
        intake.setPower(0);
        setState(State.IDLE);
        dropL.setPosition(0.55);
        dropR.setPosition(0.85);
    }
     private void checkBlock(){
        if(blockSensor.getDistance(DistanceUnit.INCH) < 2){
            off();
            isBlock = true;
        }
        else{
            isBlock = false;
        }
    }
    private void functions(){
        checkBlock();
        //if(!isBlock == false){
            //temporary buttons
            if(gamepad1.b){
                in();
            }
            else if(gamepad1.a){
                out();
            }
            else off();
        //}
        //else off();
    }
}
