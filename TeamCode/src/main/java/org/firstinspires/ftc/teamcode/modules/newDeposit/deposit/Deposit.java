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
    @Override
    public boolean isTransitioningState() {
        return platform.isTransitioningState() || lift.isTransitioningState();
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
        platform = new Platform(hardwareMap);
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
        switch (getState()) {
            case IN:
                lift.setState(Lift.Level.LEVEL1);
                break;
            case HOLDING:
            case OUT:
                lift.setState(getDefaultState());
                break;
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
