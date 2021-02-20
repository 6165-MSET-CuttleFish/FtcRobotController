package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

@Autonomous(name = "newAuto", group = "LinearOpMode")
@Disabled
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 9, 25, 0, 18, 18, telemetry, this::opModeIsActive);
        robot.autoInit();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.tiltUp();
        while(!opModeIsActive()){
            robot.scan();
            telemetry.addData("Stack Height", robot.height);
            telemetry.addData("Discs", robot.discs);
            telemetry.update();
        }
        waitForStart();
        robot.scan();
        robot.turnOffVision();
        robot.unlockIntake();
        Trajectory splineToShootingPos = robot.driveTrain.trajectoryBuilder(Robot.pwrShotLocals[0].toPose2d(0))
                .splineTo(new Vector2d(0, 0), 50)
                .build();
        robot.driveTrain.followTrajectory(splineToShootingPos, 5, () -> {
            robot.wingsIn();
            robot.launcher.setFlyWheel(0.4);
        });
        for(Coordinate powerShot: Robot.pwrShots){
            robot.driveTrain.turn(MathFunctions.AngleWrap(Coordinate.toPoint(robot.driveTrain.getPoseEstimate()).angleTo(powerShot) - robot.driveTrain.getPoseEstimate().getHeading()));
            robot.launcher.singleRound();
        }
        sleep(200);
    }
    private void goToTargetPos(){
        if(robot.discs == 4){
            case4();
        } else if(robot.discs == 1) {
            case1();
        } else {
            case0();
        }
    }
    private void case4(){
        robot.goTo(Robot.newC, 0.8, Math.toRadians(180), 0.4, ()-> {
            robot.wobbleArmDown();
            if(Robot.position.distanceTo(Robot.newC) < 5){
                robot.release();
            }
        });
        sleep(100);
        robot.wobbleArmUp();
    }
    private void case1(){
        robot.goTo(Robot.newB, 0.8, Math.toRadians(180), 0.4, ()-> {
            robot.wobbleArmDown();
            if(Robot.position.distanceTo(Robot.newB) < 5){
                robot.release();
            }
        });
        sleep(100);
        robot.wobbleArmUp();
    }
    private void case0(){
        robot.goTo(Robot.newA, 0.8, Math.toRadians(180), 0.4, ()-> {
            robot.wobbleArmDown();
            if(Robot.position.distanceTo(Robot.newA) < 5){
                robot.release();
            }
        });
        sleep(100);
        robot.wobbleArmUp();
    }
}
