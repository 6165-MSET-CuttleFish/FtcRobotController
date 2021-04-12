package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp
class webcamTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.autoInit();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Frame Count", robot.webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", robot.webcam.getFps()));
            telemetry.addData("Total frame time ms", robot.webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", robot.webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", robot.webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", robot.webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
}
