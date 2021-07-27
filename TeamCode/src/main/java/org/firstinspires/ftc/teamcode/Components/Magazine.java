package org.firstinspires.ftc.teamcode.Components;

import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Components.Details.opModeType;

public class Magazine implements Component {
    Servo magLeft1, magLeft2;
    Servo magRight1, magRight2;
    ColorRangeSensor thirdRingSensor;
    StateMachine stateMachine;
    public static double currentRings;

    public enum State {
        DOWN,
        MOVING_UP,
        MOVING_DOWN,
        UP
    }

    public Magazine(HardwareMap hardwareMap) {
        magLeft1 = hardwareMap.servo.get("magLeftBottom");
        magLeft2 = hardwareMap.servo.get("magLeftTop");
        magRight1 = hardwareMap.servo.get("magRightBottom");
        magRight2 = hardwareMap.servo.get("magRightTop");
        thirdRingSensor = hardwareMap.get(ColorRangeSensor.class, "range");
        stateMachine = new StateMachineBuilder<State>()
                .state(State.DOWN)
                .transitionTimed(0)
                .onEnter(this::down)

                .state(State.MOVING_UP)
                .transitionTimed(0.3)
                .onEnter(() -> {
                    up();
                    currentRings += 3;
                    currentRings = Range.clip(currentRings, 0, 3);
                })

                .state(State.UP)
                .transitionTimed(0.08)

                .state(State.MOVING_DOWN)
                .transitionTimed(0.38)
                .onEnter(this::down)

                .exit(State.DOWN)

                .build();
        if (opModeType == OpModeType.AUTO) {
            currentRings = 3;
        }
        down();
    }

    public double getRings() {
        if (thirdRingSensor.getDistance(DistanceUnit.INCH) < 2) {
            return 3;
        } else {
            return 2;
        }
    }

    public void up() {
        magLeft1.setPosition(0.41);
        magLeft2.setPosition(0.41);

        magRight1.setPosition(0.57);
        magRight2.setPosition(0.57);
    }

    public void down() {
        magLeft1.setPosition(0.75);
        magLeft2.setPosition(0.75);

        magRight1.setPosition(0.27);
        magRight2.setPosition(0.27);
    }

    public State getState() {
        return (State) stateMachine.getState();
    }

    public void magMacro() {
        if (!stateMachine.getRunning()) stateMachine.start();
    }

    public void update() {
        stateMachine.update();
    }
}
