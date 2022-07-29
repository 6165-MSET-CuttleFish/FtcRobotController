package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerext.drive.ImprovedDrive;

public class StaticConfig {
    public static ImprovedDrive getDrive(HardwareMap hardwareMap) {
        return new SampleMecanumDrive(hardwareMap);
    }
}
