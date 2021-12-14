package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//http://192.168.43.1:8080/dash
@TeleOp
@Disabled
public class ModuleTestOpMode extends OpMode {
    Module module;
    @Override
    public void init() {
        module = new Module(hardwareMap, null) {
            @Override
            public void init() {

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
    }

    @Override
    public void loop() {
        module.update();
    }
}
