package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.PIDController;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import java.io.DataInput;
import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ShooterTest", group = "LinearOpMode")
public class ShooterTest extends LinearOpMode
{
    Robot robot;
    Runnable wingDefault;
    boolean ninja, armUp;
    Coordinate shootingPosition;
    double shootingAngle = 0;
    boolean isWingsOut = false;
    double velo = 0;
    int wingsLimit = 0;
    ArrayList<Double> timer = new ArrayList<Double> ();
    double currentMillis = 0;
    double lastMillis = 0;
    int cycles = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        //robot = new Robot (hardwareMap, telemetry, () -> opModeIsActive() && gamepadIdle());

       // robot = new Robot (hardwareMap, telemetry, () -> opModeIsActive() && gamepadIdle());

        robot.init();
        //wingDefault = ()->robot.launcher.wingsVert();
        // pidRotate = new PIDController(.07, 0.014, 0.0044);
        //Thread driveTrain = new Thread(this);
        waitForStart();
    }

}
