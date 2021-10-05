package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.modules.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

/**
 * Frontal mechanism for collecting freight
 *
 * @author Matthew Song
 */
public class Intake extends Module<Intake.State> {
    private DcMotor intake;
    private Servo blocker;
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        blockSensor = hardwareMap.get(DistanceSensor.class, "block");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        isBlock = false;
    }

    @Override
    public State getState() {

        return this.getState();
    }

    /**
     * @return Whether the module is currently in a potentially hazardous state for autonomous to resume
     */
    @Override
    public boolean isHazardous() {
        return false;
    }

    @Override
    public void update() {
        functions();
    }
    private void in(){
        intake.setPower(1);
        setState(State.INTAKING);
    }
    private void out(){
        intake.setPower(-1);
        setState(State.EXTAKING);
    }
    private void off(){
        intake.setPower(0);
        setState(State.IDLE);
    }
    /* private void checkBlock(){
        if(DistanceSensor.distanceOutOfRange){
            off();
            isBlock = true;
        }
    } */
    private void functions(){
        //checkBlock();
        if(!isBlock == false){
            if(/*gamepad1.*/){
                in();
            }
            else if(/*gamepad1.*/){
                out();
            }
            else off();
        }
    }
}
