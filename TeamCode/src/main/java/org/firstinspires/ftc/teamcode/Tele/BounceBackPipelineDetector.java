package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
        robot = new Robot(this);
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

        camera.setPipeline(pipeline = new RingLocalizer());

        RingLocalizer.CAMERA_WIDTH = CAMERA_WIDTH;

        RingLocalizer.HORIZON = HORIZON;

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        robot.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.y) robot.followTrajectory(pickup(Math.toRadians(robot.getPoseEstimate().getHeading())));
            //robot.ringUpdate(pipeline.getVectors(robot.getPoseEstimate()));
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            telemetry.addData("x", pipeline.getX());
            telemetry.addData("y", pipeline.getY());
        }
    }
    Trajectory pickup(double endTangent){
        TrajectoryBuilder builder = robot.trajectoryBuilder(robot.getPoseEstimate())
                .addDisplacementMarker(()-> robot.intake(1));
        Vector2d[] wayPoints = pipeline.getVectors(robot.getPoseEstimate()).toArray(new Vector2d[0]).clone();
        bubbleSort(wayPoints);
        for(int i = 0; i < wayPoints.length; i++){
            builder = builder.splineTo(wayPoints[i], i < wayPoints.length - 2 ? wayPoints[i].angleBetween(wayPoints[i + 1]) : endTangent);
        }
        return builder.addDisplacementMarker(()-> robot.intake(0)).build();
    }
    void bubbleSort(Vector2d[] arr)
    {
        int n = arr.length;
        for (int i = 0; i < n-1; i++)
            for (int j = 0; j < n-i-1; j++)
                if (arr[j].distTo(robot.getPoseEstimate().vec()) > arr[j+1].distTo(robot.getPoseEstimate().vec()))
                {
                    // swap arr[j+1] and arr[j]
                    Coordinate temp = Coordinate.toPoint(arr[j]);
                    arr[j] = arr[j+1];
                    arr[j+1] = temp.toVector();
                }
    }
}