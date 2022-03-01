package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class HubDetectionTest extends ModuleTest {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    public OpenCvWebcam webcam;
    private ShippingHubDetector shippingHubDetector;
    Robot robot;

    @Override
    public void initialize() {
        robot = new Robot(this);
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(shippingHubDetector = new ShippingHubDetector());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
            }
        });
        dashboard.startCameraStream(webcam, 30);

        telemetry.addLine("waiting for start");
        telemetry.update();
    }

    @Override
    public void update() {
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        0,
                        -gamepad1.right_stick_x
                )
        );
        telemetry.addData("X", shippingHubDetector.getX());
        telemetry.addData("Y", shippingHubDetector.getY());
    }
}
