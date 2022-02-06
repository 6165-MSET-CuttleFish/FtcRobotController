package org.firstinspires.ftc.teamcode.modules.capstone;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.wrappers.ControllableServos;

@Config
public class Capstone extends Module<Capstone.State> {
    public static double servoIncrementHorizontal = 0.0001, servoIncrementVertical = 0.0001;
    public static double horizontalTolerance = 0.5, verticalTolerance = 0.2;
    public static double servoIncrementHorizontalLarge = 0.001, servoIncrementVerticalLarge = 0.01;
    private double horizontalPos = 0.5, verticalPos;
    private CRServo tape;
    private ControllableServos verticalTurret, horizontalTurret;

    @Override
    public boolean isTransitioningState() {
        return false;
    }

    public enum State implements StateBuilder {
        READY,
        PICKING_UP,
        HOLDING,
        PRECAP,
        CAPPING;

        @Override
        public Double getTimeOut() {
            return null;
        }
    }

    /**
     * @param hardwareMap  instance of the hardware map provided by the OpMode
     */
    public Capstone(HardwareMap hardwareMap) {
        super(hardwareMap, State.HOLDING);
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
    }

    public void setHorizontalTurret(double pwr) {
        if (pwr > horizontalTolerance) horizontalPos += servoIncrementHorizontal;
        else if (pwr < -horizontalTolerance) horizontalPos -= servoIncrementHorizontal;
        horizontalTurret.setPosition(horizontalPos);
    }
    public void incrementHorizontal(double pwr) {
        if (pwr > horizontalTolerance) horizontalPos += servoIncrementHorizontalLarge;
        else if (pwr < -horizontalTolerance) horizontalPos -= servoIncrementHorizontalLarge;
        horizontalTurret.setPosition(horizontalPos);
    }
    public void incrementVertical(double pwr) {
        if (pwr > -verticalTolerance) verticalPos += servoIncrementVerticalLarge;
        else if (pwr < verticalTolerance) verticalPos -= servoIncrementVerticalLarge;
        verticalTurret.setPosition(verticalPos);
    }
    public void setVerticalTurret(double pwr) {
        if (pwr < -verticalTolerance) verticalPos += servoIncrementVertical;
        else if (pwr > verticalTolerance) verticalPos -= servoIncrementVertical;
        verticalTurret.setPosition(verticalPos);
    }
    public void setTape(double pwr) {
        tape.setPower(pwr);
    }
    public boolean isDoingInternalWork() {
        return false;
    }
}
