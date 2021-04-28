package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.RingLocalizer;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "BounceBackPipeline", group = "LinearOpMode")
public class BounceBackPipelineDetector extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 60; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private RingLocalizer pipeline;
    private OpenCvCamera camera;
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 0, 0, 0);
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new RingLocalizer(this));

        RingLocalizer.CAMERA_WIDTH = CAMERA_WIDTH;

        RingLocalizer.HORIZON = HORIZON;

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        robot.driveTrain.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            robot.driveTrain.followTrajectory(pickup());
            telemetry.addData("X", pipeline.getX());
            telemetry.addData("Y", pipeline.getY());
            robot.driveTrain.ringUpdate(pipeline.getVectors(robot.driveTrain.getPoseEstimate()));
            telemetry.update();
        }
    }
    Trajectory pickup(){
        TrajectoryBuilder builder = robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                .addDisplacementMarker(()-> robot.intake(1));
        Vector2d[] wayPoints = pipeline.getVectors(robot.driveTrain.getPoseEstimate()).toArray(new Vector2d[0]);
        bubbleSort(wayPoints);
        for(Vector2d wayPoint: wayPoints){
            builder = builder.splineTo(wayPoint, 0);
        }
        return builder.addDisplacementMarker(()-> robot.intake(0)).build();
    }
    void bubbleSort(Vector2d[] arr)
    {
        int n = arr.length;
        for (int i = 0; i < n-1; i++)
            for (int j = 0; j < n-i-1; j++)
                if (arr[j].getY() < arr[j+1].getY())
                {
                    // swap arr[j+1] and arr[j]
                    Coordinate temp = Coordinate.toPoint(arr[j]);
                    arr[j] = arr[j+1];
                    arr[j+1] = temp.toVector();
                }
    }
}