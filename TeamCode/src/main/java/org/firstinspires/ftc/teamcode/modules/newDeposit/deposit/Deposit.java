package org.firstinspires.ftc.teamcode.modules.newDeposit.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.modules.StateBuilder;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Context;

/**
 * Deposit Module Wrapper
 * @author Ayush Raman
 */
@Config
public class Deposit extends Module<Deposit.State> {
    public static boolean farDeposit = true;
    public static boolean allowLift = true;
    public static double LEVEL3 = 11.8;
    public static double LEVEL2 = 0;
    public static double LEVEL1 = 0;

    private double getLevelHeight(Lift.Level state) {
        switch (state) {
            case LEVEL3: return LEVEL3;
            case LEVEL2: return LEVEL2;
            case LEVEL1: return LEVEL1;
        }
        return 0;
    }

    @Override
    public boolean isTransitioningState() {
        return false;
    }

    public enum State implements StateBuilder {
        OUT,
        HOLDING,
        IN,
        DUMPING;
        @Override
        public Double getTimeOut() {
            return null;
        }
    }

    public Lift lift;
    public Platform platform;
    private Intake intake;

    /**
     * @param hardwareMap instance of the hardware map provided by the OpMode
     * @param intake instance of robot's intake module
     */
    public Deposit(HardwareMap hardwareMap, Intake intake) {
        super(hardwareMap, State.IN, new Pose2d());
        this.intake = intake;
        lift = new Lift(hardwareMap);
        platform = new Platform(hardwareMap, intake, this);
        setNestedModules(platform, lift);
    }

    /**
     * This function initializes all necessary hardware modules
     */
    @Override
    public void internalInit() {
    }

    private Lift.Level defaultState = Lift.Level.LEVEL3;

    public Lift.Level getDefaultState() {
        return defaultState;
    }

    public void setDefaultLevel(Lift.Level level) {
        defaultState = level;
    }

    public void dump() {
       platform.dump();
    }

    /**
     * This function updates all necessary controls in a loop
     */
    @Override
    public void internalUpdate() {
        Vector2d moduleVec = lift.getModuleVector().plus(platform.getModuleVector());
        Vector2d moduleVecFactoringPitch = moduleVec.rotated(Context.pitch);
        if (platform.isDoingWork() && allowLift) {
            lift.setState(getDefaultState());
        } else {
            lift.setState(Lift.Level.LEVEL1);
        }
        switch (getState()) {
            case HOLDING:
                //platform.hold();
                break;
            case IN:
            case OUT:
                lift.setState(getDefaultState());
        }
        if (isDebugMode()) {
            Context.packet.put("Deposit X", moduleVec.getX());
            Context.packet.put("Deposit Y", moduleVec.getY());
        }
    }
      
    /**
     * @return Whether the module is currently doing work for which the robot must remain stationary for
     */
    public boolean isDoingInternalWork() {
        return false;
    }
}
