package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@Autonomous
public class testAuto extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 79, 56.5, 0,18, 18, this::opModeIsActive);
        robot.init();
        telemetry.addData("orient", robot.position.returnOrientation());
        telemetry.addData("x",robot.position.x);
        telemetry.addData("y", robot.position.y);
        telemetry.update();
        waitForStart();
        for(int i = 0; i < 3; i++){
            robot.turnTo(Robot.pwrShots[i], 0.3);
        }
        //robot.orient(0, 0.4);
    }
}
