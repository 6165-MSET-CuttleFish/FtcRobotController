package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;
import com.noahbres.jotai.StateMachine;
import com.noahbres.jotai.StateMachineBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class TurretTuner {
    public static double MAX_ANGLE = 360;
    public static double MIN_ANGLE = 0;


    // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
    // alphabetical order. Thus, we preserve the actual order of the process
    // Then we append Z just because we want it to show below the MOTOR_ and TESTING_ because
    // these settings aren't as important
    public static double ZSTATE1_RAMPING_UP_DURATION = 2;
    public static double ZSTATE2_COASTING_1_DURATION = 4;
    public static double ZSTATE3_RAMPING_DOWN_DURATION = 2;
    public static double ZSTATE4_COASTING_2_DURATION = 2;
    public static double ZSTATE5_RANDOM_1_DURATION = 2;
    public static double ZSTATE6_RANDOM_2_DURATION = 2;
    public static double ZSTATE7_RANDOM_3_DURATION = 2;
    public static double ZSTATE8_REST_DURATION = 1;

    enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    private StateMachine stateMachine;

    private ElapsedTime externalTimer = new ElapsedTime();

    private double currentTargetAngle = 0.0;

    public TurretTuner() {
        stateMachine = new StateMachineBuilder<State>()
                .state(State.RAMPING_UP)
                .transitionTimed(ZSTATE1_RAMPING_UP_DURATION)
                .onEnter(externalTimer::reset)
                .loop(() -> {
                    double progress = externalTimer.seconds() / ZSTATE1_RAMPING_UP_DURATION;
                    currentTargetAngle = progress * (MAX_ANGLE - MIN_ANGLE) + MAX_ANGLE;
                })

                .state(State.COASTING_1)
                .transitionTimed(ZSTATE2_COASTING_1_DURATION)
                .onEnter(() -> currentTargetAngle = MAX_ANGLE)

                .state(State.RAMPING_DOWN)
                .transitionTimed(ZSTATE3_RAMPING_DOWN_DURATION)
                .onEnter(externalTimer::reset)
                .loop(() -> {
                    double progress = externalTimer.seconds() / ZSTATE3_RAMPING_DOWN_DURATION;
                    currentTargetAngle = MAX_ANGLE - progress * (MAX_ANGLE - MIN_ANGLE);
                })

                .state(State.COASTING_2)
                .transitionTimed(ZSTATE4_COASTING_2_DURATION)
                .onEnter(() -> currentTargetAngle = MIN_ANGLE)

                .state(State.RANDOM_1)
                .transitionTimed(ZSTATE5_RANDOM_1_DURATION)
                .onEnter(() -> currentTargetAngle = Math.random() * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE)

                .state(State.RANDOM_2)
                .transitionTimed(ZSTATE6_RANDOM_2_DURATION)
                .onEnter(() -> currentTargetAngle = Math.random() * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE)

                .state(State.RANDOM_3)
                .transitionTimed(ZSTATE7_RANDOM_3_DURATION)
                .onEnter(() -> currentTargetAngle = Math.random() * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE)

                .state(State.REST)
                .transitionTimed(ZSTATE8_REST_DURATION)
                .onEnter(() -> currentTargetAngle = 0)

                .exit(State.RAMPING_UP)

                .build();

        stateMachine.setLooping(true);
    }

    public void start() {
        externalTimer.reset();
        stateMachine.start();
    }

    public double update() {
        stateMachine.update();

        return currentTargetAngle;
    }
    public boolean getRunning() {
        return stateMachine.getRunning();
    }
}