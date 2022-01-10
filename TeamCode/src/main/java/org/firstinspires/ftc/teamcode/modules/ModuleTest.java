package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Balance;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.field.Side;

import java.util.List;

public abstract class ModuleTest extends OpMode {
    private Module[] modules = {};
    private KeyReader[] keyReaders = {};
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    private List<LynxModule> allHubs;
    public abstract void initialize();

    public abstract void update();

    public void setModules(Module... modules) {
        this.modules = modules;
    }

    public void setKeyReaders(KeyReader... keyReaders) {
        this.keyReaders = keyReaders;
    }

    @Override
    public final void init() {
        initialize();
        for (Module module : modules) {
            module.init();
            module.setDebugMode(true);
        }
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        Context.telemetry = telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Context.opModeType = OpModeType.NONE;
        Context.side = Side.NONE;
        Context.robotPose = new Pose2d();
        Context.alliance = Alliance.NONE;
        Context.balance = Balance.BALANCED;
    }

    @Override
    public final void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        for (KeyReader keyReader : keyReaders) {
            keyReader.readValue();
        }
        for (Module module : modules) {
            module.update();
        }
        update();
        assert Context.telemetry != null;
        dashboard.sendTelemetryPacket(Context.packet);
        Context.packet = new TelemetryPacket();
    }
}
