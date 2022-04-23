package org.firstinspires.ftc.teamcode.modules.capstone;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;
import org.firstinspires.ftc.teamcode.util.field.Context;

@Config
public class Capstone extends Module<Capstone.State> {
    public static double servoIncrementHorizontal = 0.0003, servoIncrementVertical = -0.00014;
    public static double horizontalTolerance = 0, verticalTolerance = 0;
    public static double servoIncrementHorizontalLarge = 0.01, servoIncrementVerticalLarge = 0.01;
    private double horizontalPos = 0.5, verticalPos = 0.45;
    public static double passivePower = 0.0;
    private CRServo tape;
    private Servo verticalTurret, horizontalTurret;
    public static double verticalPosDef = 0.5, horizontalPosDef = 0.0;
    private double verticalInc, horizontalInc;
    public static double vUpperLimit = 0.65, vLowerLimit = 0.32;
    public static double hUpperLimit = 1.0, hLowerLimit = 0.0;

    @Override
    public boolean isTransitioningState() {
        return false;
    }

    public enum State implements StateBuilder {
        IDLE,
        AUTORETRACT,
        ACTIVE,
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
        super(hardwareMap, State.IDLE, new Pose2d());
    }

    public void internalInit() {
        tape = hardwareMap.crservo.get("tape");
        tape.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalTurret = hardwareMap.servo.get("hTurret");
        verticalTurret = hardwareMap.servo.get("vTurret");
    }

    double power;

    @Override
    protected void internalUpdate() {
        double millisSinceLastUpdate = getMillisecondsSinceLastUpdate();
        verticalPos = Range.clip(verticalPos + (verticalInc * millisSinceLastUpdate), vLowerLimit, vUpperLimit);
        horizontalPos = Range.clip(horizontalPos + (horizontalInc * millisSinceLastUpdate), hLowerLimit, hUpperLimit);
        switch (getState()) {
            case IDLE:
                tape.setPower(passivePower);
                verticalTurret.setPosition(verticalPosDef);
                horizontalTurret.setPosition(horizontalPosDef);
                break;
            case ACTIVE:
                tape.setPower(power);
                verticalTurret.setPosition(verticalPos);
                horizontalTurret.setPosition(horizontalPos);
                break;
            case AUTORETRACT:
                tape.setPower(-1);
                verticalTurret.setPosition(verticalPosDef);
                if (getSecondsSpentInState() > 2.5) {
                    horizontalTurret.setPosition(horizontalPosDef);
                    tape.setPower(0.0);
                }
                break;
        }
        verticalInc = 0;
        horizontalInc = 0;
        if (isDebugMode()) {
            Context.packet.put("Horizontal Turret", horizontalTurret.getPosition());
            Context.packet.put("Vertical Turret", verticalTurret.getPosition());
        }
    }

    public void setHorizontalTurret(double pwr) {
        if (Math.abs(pwr) > horizontalTolerance) horizontalInc = servoIncrementHorizontal * pwr;
    }
    public void incrementHorizontal(double pwr) {
        if (Math.abs(pwr) > horizontalTolerance) horizontalPos += servoIncrementHorizontalLarge * pwr;
    }
    public void setHorizontalAngle(double angle) {
//        horizontalTurret.setAngle(angle);
//        horizontalPos = horizontalTurret.getPosition();
    }
    public void incrementVertical(double pwr) {
        if (Math.abs(pwr) > verticalTolerance) verticalPos += servoIncrementVerticalLarge * pwr;
    }
    public void setVerticalTurret(double pwr) {
        if (Math.abs(pwr) > verticalTolerance) verticalInc = servoIncrementVertical * pwr;
    }
    public void setVerticalAngle(double angle) {
//        verticalTurret.setAngle(angle);
//        verticalPos = verticalTurret.getPosition();
    }
    public void setTape(double pwr) {
        power = pwr;
    }
    public boolean isDoingInternalWork() {
        return false;
    }

    public void retract() {
        setState(State.AUTORETRACT);
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void active() {
        setState(State.ACTIVE);
    }
}
