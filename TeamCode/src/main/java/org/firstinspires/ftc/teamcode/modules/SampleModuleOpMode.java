package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//http://192.168.43.1:8080/dash
@TeleOp
@Disabled
public class SampleModuleOpMode extends ModuleTest {
    Module module;
    @Override
    public void initialize() {
        module = new Module(hardwareMap, null) {
            @Override
            public void internalInit() {

            }

            @Override
            public void internalUpdate() {

            }

            @Override
            public boolean isDoingInternalWork() {
                return false;
            }

            @Override
            public boolean isModuleInternalHazardous() {
                return false;
            }
        };
        setModules(module);
    }

    @Override
    public void update() {
        module.isHazardous();
    }
}
