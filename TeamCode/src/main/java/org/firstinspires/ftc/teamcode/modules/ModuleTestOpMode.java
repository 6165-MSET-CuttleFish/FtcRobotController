package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;

//http://192.168.43.1:8080/dash
@TeleOp
@Disabled
public class ModuleTestOpMode extends ModuleTest {
    Module module;
    @Override
    public void init() {
        module = new Module(hardwareMap, null) {
            @Override
            public void init() {

            }

            @Override
            public void update() {

            }

            @Override
            public boolean isDoingWork() {
                return false;
            }

            @Override
            public boolean isHazardous() {
                return false;
            }
        };
        init(module);
    }

    @Override
    public void loop() {
        update();
    }
}
