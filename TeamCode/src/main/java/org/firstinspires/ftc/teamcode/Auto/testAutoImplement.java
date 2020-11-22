package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class testAutoImplement extends LinearOpMode{
    //Robot robot;
    BNO055IMU imu;
    public void runOpMode() throws InterruptedException{
        //robot = new Robot(DcMotor.RunMode.RUN_USING_ENCODER, hardwareMap, 0, 0, 18, 18);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("first",imu.getAngularOrientation().firstAngle);
            telemetry.addData("second",imu.getAngularOrientation().secondAngle);
            telemetry.addData("third",imu.getAngularOrientation().thirdAngle);
            telemetry.update();
        }
    }
}
