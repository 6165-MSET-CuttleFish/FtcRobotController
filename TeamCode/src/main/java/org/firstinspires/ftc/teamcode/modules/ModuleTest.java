package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.field.Side;

public abstract class ModuleTest extends OpMode {

    private Module[] modules;

    public abstract void initialize();

    public abstract void update();

    public void setModules(Module... modules) {
        this.modules = modules;
    }

    @Override
    public final void init() {
        initialize();
        Context.telemetry = telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Context.opModeType = OpModeType.NONE;
        Context.side = Side.NONE;
    }

    @Override
    public final void loop() {
        for (Module module : modules) {
            module.update();
        }
        update();
    }
}
