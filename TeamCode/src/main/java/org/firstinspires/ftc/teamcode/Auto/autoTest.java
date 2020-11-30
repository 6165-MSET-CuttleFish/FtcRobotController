package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@Autonomous
public class autoTest extends LinearOpMode {
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
        robot.pidRotate(90, 0.5);
        telemetry.addData("Done", "");
        telemetry.update();
        robot.position.stop();
    }
    public void foo(){
        //robot.goToPosition(new Coordinate(28, 24), 0.4, 0, 2);
        robot.setMovement(0, 0.3, 0);
        while(opModeIsActive()){
            telemetry.addData("x",robot.position.x);
            telemetry.addData("y", robot.position.y);
            telemetry.addData("angle", robot.position.radians());
            telemetry.update();
        }
    }
    public void boo(){
        robot.setMovement(0, 0, 0);
    }
    public void soo(){
        goTo(new Coordinate(28, 24), 0.4, 0, 0);
        sleep(1000);
        goTo(new Coordinate(40, 12), 0.4, Math.toRadians(50), 0.5);
        sleep(1000);
        goTo(new Coordinate(14, 24), 0.4, 0, 0.2);
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed){
        double distance = Math.hypot(pt.x - robot.position.getX(), pt.y - robot.position.y);
        while(distance > 4) {
            distance = Math.hypot(pt.x - robot.position.x, pt.y - robot.position.y);

            double absAngleToTarget = Math.atan2(pt.y - robot.position.y, pt.x - robot.position.x);
            telemetry.addData("x",robot.position.x);
            telemetry.addData("y", robot.position.y);
            telemetry.addData("angle", robot.position.radians());
            telemetry.addData("distance", distance);
            telemetry.update();
            double relAngleToPoint = AngleWrap(absAngleToTarget - robot.position.radians() + Math.toRadians(90));
            //System.out.println("Rel " + relAngleToPoint);
            double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
            double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            double movement_x = movementXPower * power;
            double movement_y = movementYPower * power;
            double relTurnAngle = relAngleToPoint - Math.toRadians(90) + preferredAngle;
            double movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
            robot.setMovement(movement_x, movement_y, -movement_turn);
        }
        robot.setMovement(0, 0, 0);
    }
}
