package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@Autonomous
@Disabled
public class testAuto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap, 9, 25, 0);
        robot.autoInit();
        telemetry.addData("orient", robot.position.returnOrientation());
        telemetry.addData("x",robot.position.x);
        telemetry.addData("y", robot.position.y);
        telemetry.update();
        waitForStart();
       // robot.goTo(Robot.leftWobble, 0.6, Math.toRadians(0), 0.4);
        //robot.orient(0, 0.4);
    }
}
