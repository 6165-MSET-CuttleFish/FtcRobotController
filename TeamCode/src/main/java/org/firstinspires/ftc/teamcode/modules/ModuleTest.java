package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.field.Side;

import java.util.List;

public abstract class ModuleTest extends OpMode {

    private Module[] modules;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private List<LynxModule> allHubs;
    public abstract void initialize();

    public abstract void update();

    public void setModules(Module... modules) {
        this.modules = modules;
    }

    @Override
    public final void init() {
        initialize();
        allHubs = hardwareMap.getAll(LynxModule.class);
        Context.telemetry = telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Context.opModeType = OpModeType.NONE;
        Context.side = Side.NONE;
    }

    @Override
    public final void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        for (Module module : modules) {
            module.update();
        }
        update();
        assert Context.telemetry != null;
        //Context.telemetry.update();
        dashboard.sendTelemetryPacket(Context.packet);
        Context.packet = new TelemetryPacket();
    }
}
