package org.firstinspires.ftc.teamcode.modules;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//http://192.168.43.1:8080/dash
@TeleOp
@Disabled
public class SampleModuleOpMode extends ModuleTest {
    Module<StateBuilder> module;
    KeyReader keyReader;
    @Override
    public void initialize() {
        module = new Module<StateBuilder>(hardwareMap, new StateBuilder() {
            @Override
            public double getTimeOut() {
                return 0;
            }

            @Override
            public double getPercentMotion() {
                return 0;
            }
        }) {
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
        keyReader = new ButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        setModules(module);
        setKeyReaders(keyReader);
    }

    @Override
    public void update() {
        module.isHazardous();
    }
}
