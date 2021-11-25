package org.firstinspires.ftc.teamcode.util.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.field.Details;

import static org.firstinspires.ftc.teamcode.util.field.Details.packet;

public abstract class ModuleTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Module[] modules;

    public void init(Module... modules) {
        this.modules = modules;
        Details.telemetry = telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    public void update() {
        printStates(modules);
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    private void printStates(Module... modules) {
        for (Module module : modules) {
            module.update();
            telemetry.addData(module.getClass().getSimpleName() +" State", module.getState());
            if (module.nestedModules.length != 0) printStates(module.nestedModules);
        }
    }
}
