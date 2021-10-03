package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.modules.Module;

/**
 * Frontal mechanism for collecting freight
 *
 * @author Matthew Song
 */
public class Intake extends Module<Intake.State> {
    private DcMotor intake;
    private Servo blocker;
    enum State {
        INTAKING,
        EXTAKING,
        OFF,
        HAS_OBJECT
    }

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        // TODO: update control loops and modules
    }
    public void in(){
        intake.setPower(1);
        setState(State.INTAKING);
    }
    public void out(){
        intake.setPower(-1);
        setState(State.EXTAKING);
    }
    public void off(){
        intake.setPower(0);
        setState(State.OFF);
    }
}
