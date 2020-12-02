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
        robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap, 14, 24, 0,18, 18);
        robot.init();
        telemetry.addData("orient", robot.position.returnOrientation());
        telemetry.addData("x",robot.position.x);
        telemetry.addData("y", robot.position.y);
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            robot.pidRotate(2, 0.5);
            telemetry.addData("2 degrees","");
            telemetry.update();
            sleep(1000);
            robot.pidRotate(90, 0.5);
            telemetry.addData("2 degrees","");
            telemetry.update();
            sleep(1000);
        }
        robot.position.stop();
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed){
        double distance = Math.hypot(pt.x - robot.position.getX(), pt.y - robot.position.y);
        while(distance > 5) {
            distance = Math.hypot(pt.x - robot.position.x, pt.y - robot.position.y);

            double absAngleToTarget = Math.atan2(pt.y - robot.position.y, pt.x - robot.position.x);

            double relAngleToPoint = AngleWrap(absAngleToTarget - robot.position.radians());
            //System.out.println("Rel " + relAngleToPoint);
            double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
            double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            double movement_x = movementXPower * power;
            double movement_y = movementYPower * power;
            double relTurnAngle = relAngleToPoint + preferredAngle;
            telemetry.addData("RelAngleToPt", relAngleToPoint);
            telemetry.addData("TurnAngle", relTurnAngle);
            telemetry.update();
            double movement_turn = distance > 5 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
            //double movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
            robot.setMovement(movement_x, movement_y, -movement_turn);
        }
        robot.setMovement(0, 0, 0);
    }
}
