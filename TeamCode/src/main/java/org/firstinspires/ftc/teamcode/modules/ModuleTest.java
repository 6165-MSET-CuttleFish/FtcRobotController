package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.field.Side;

public abstract class ModuleTest extends OpMode {

    private Module[] modules;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public abstract void initialize();

    public abstract void update();

    public void setModules(Module... modules) {
        this.modules = modules;
    }

    @Override
    public final void init() {
        initialize();
        Context.telemetry = telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Context.opModeType = OpModeType.NONE;
        Context.side = Side.NONE;
    }

    @Override
    public final void loop() {
        for (Module module : modules) {
            module.update();
        }
        update();
        assert Context.telemetry != null;
        Context.telemetry.update();
        dashboard.sendTelemetryPacket(Context.packet);
        Context.packet = new TelemetryPacket();
    }
}
