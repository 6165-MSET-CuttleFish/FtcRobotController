package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.Side;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;

public class BlueStackAuto extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    Gunner gunner;
    Turret turret;
    Magazine magazine;
    Intake intake;
    WobbleArm wobbleArm;
    Claw claw;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, new Pose2d(-61.5975, 27.64, 0), OpModeType.AUTO, Side.BLUE);
        shooter = robot.shooter;
        intake = robot.intake;
        wobbleArm = robot.wobbleArm;
        turret = shooter.turret;
        gunner = shooter.gunner;
        magazine = shooter.magazine;
        claw = wobbleArm.claw;
        waitForStart();
    }
}
