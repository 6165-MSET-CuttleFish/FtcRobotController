package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "visionTest")
public class TestOpMode extends ModuleTest {

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    public OpenCvWebcam webcam;
    private Detector detector;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        init(new Module(hardwareMap, null) {
            @Override
            public void init() {

            }

            @Override
            public void update() {

            }

            @Override
            public boolean isDoingWork() {
                return false;
            }

            @Override
            public boolean isHazardous() {
                return false;
            }
        });

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

        webcam.setPipeline(detector = new Detector());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        dashboard.startCameraStream(webcam, 30);

        telemetry.addLine("waiting for start");
        telemetry.update();
    }

    @Override
    public void loop() {
        update();
        telemetry.addData("Location", detector.getLocation());
        telemetry.update();
    }
}
