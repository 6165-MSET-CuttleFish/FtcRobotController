package org.firstinspires.ftc.teamcode.modules.capstone;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;

@Config
public class Capstone extends Module<Capstone.State> {
    public static double servoIncrementHorizontal = 0.007, servoIncrementVertical = -0.01;
    public static double horizontalTolerance = 0, verticalTolerance = 0;
    public static double servoIncrementHorizontalLarge = 0.01, servoIncrementVerticalLarge = 0.01;
    private double horizontalPos = 0.5, verticalPos = 0.55;
    private CRServo tape;
    private ControllableServos verticalTurret, horizontalTurret;

    @Override
    public boolean isTransitioningState() {
        return false;
    }

    public enum State implements StateBuilder {
        IDLE,
        AUTORETRACT,
        ;

        @Override
        public Double getTimeOut() {
            return null;
        }
    }

    /**
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     */
    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.IDLE);
    }

    public void internalInit() {
        tape = hardwareMap.crservo.get("tape");
        tape.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalTurret = new ControllableServos(hardwareMap.servo.get("hTurret"));
        verticalTurret = new ControllableServos(hardwareMap.servo.get("vTurret"));
        setActuators(horizontalTurret, verticalTurret);
    }

    @Override
    protected void internalUpdate() {
        verticalPos = Range.clip(verticalPos, 0, 1);
        horizontalPos = Range.clip(horizontalPos, 0, 1);
        verticalTurret.setPosition(verticalPos);
        horizontalTurret.setPosition(horizontalPos);
    }

    public void setHorizontalTurret(double pwr) {
        if (Math.abs(pwr) > horizontalTolerance) horizontalPos += servoIncrementHorizontal * pwr;

    }
    public void incrementHorizontal(double pwr) {
        if (Math.abs(pwr) > horizontalTolerance) horizontalPos += servoIncrementHorizontalLarge * pwr;
    }
    public void incrementVertical(double pwr) {
        if (Math.abs(pwr) > verticalTolerance) verticalPos += servoIncrementVerticalLarge * pwr;
    }
    public void setVerticalTurret(double pwr) {
        if (Math.abs(pwr) > verticalTolerance) verticalPos += servoIncrementVertical * pwr;
    }
    public void setTape(double pwr) {
        tape.setPower(pwr);
    }
    public boolean isDoingInternalWork() {
        return false;
    }
}
