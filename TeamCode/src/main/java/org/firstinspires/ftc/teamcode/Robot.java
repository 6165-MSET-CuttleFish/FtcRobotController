package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.drive.FrequentPositions;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.roadrunnerext.drive.ImprovedDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.field.Freight;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.field.Context.opModeType;
import static org.firstinspires.ftc.teamcode.util.field.Context.robotPose;
import static org.firstinspires.ftc.teamcode.util.field.Context.alliance;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.util.field.Context.telemetry;
import static org.firstinspires.ftc.teamcode.util.field.Context.totalCharge;
import static org.firstinspires.ftc.teamcode.util.field.Context.totalCurrent;

/**
 * This class represents the robot and its drivetrain
 * @author Ayush Raman
 */
@Config
public class Robot<T> {
    /*
     * Robot statics
     */
    public static double MAX_CURRENT = 15;
    public static double MID_CHARGE = 12;
    public static double MAX_CHARGE = 40;
    public static double COOLDOWN_TIME = 0.4;
    public static Pose2d admissibleError = new Pose2d(6, 6, Math.toRadians(10));
    public static Pose2d admissibleVelo = new Pose2d(5, 5, Math.toRadians(60));
    public static double admissibleTimeout = 0.3;
    public static boolean isDebugMode;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam
    private OpenCvCamera webcam;

    final HardwareMap hardwareMap;

    public ImprovedDrive driveTrain;
    private final Module[] modules;

    private final VoltageSensor batteryVoltageSensor;
    private final List<LynxModule> allHubs;

    private final FtcDashboard dashboard;

    public Robot(OpMode opMode, Pose2d pose2d) {
        this(opMode, pose2d, OpModeType.NONE, Alliance.NONE);
    }

    public Robot(OpMode opMode, OpModeType type, Alliance alliance) {
        this(opMode, Context.robotPose, type, alliance);
    }

    public Robot(OpMode opMode, OpModeType type) {
        this(opMode, Context.robotPose, type, alliance);
    }

    public Robot(OpMode opMode) {
        this(opMode, new Pose2d(0, 0, 0), OpModeType.NONE, Alliance.NONE);
    }

    public Robot(OpMode opMode, Pose2d pose2d, OpModeType type, Alliance alliance) {
        dashboard = FtcDashboard.getInstance();
        Context.opModeType = type;
        Context.alliance = alliance;
        Context.freight = Freight.NONE;
        robotPose = pose2d;
        if (opModeType == OpModeType.AUTO) robotPose = FrequentPositions.startingPosition();
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        dashboard.setTelemetryTransmissionInterval(25);
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        modules = new Module[] {
                driveTrain = new SampleTankDrive(hardwareMap),
                // TODO: initialize other modules here
        };
        for (Module module : modules) {
            module.init();
        }
        driveTrain.setPoseEstimate(robotPose);
        telemetry.clear();
        telemetry.addData("Init", "Complete");
        telemetry.update();
    }



    public void visionInit() {
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
                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {
            }
        });
        dashboard.startCameraStream(webcam, 30);
    }

    public void setPipeline(OpenCvPipeline pipeline) {
        webcam.setPipeline(pipeline);
    }

    public void turnOffVision() {
        dashboard.stopCameraStream();
        webcam.closeCameraDeviceAsync(() -> {

        });
    }

    ElapsedTime loopTime = new ElapsedTime();

    public void update() {
        totalCurrent = 0;
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
            totalCurrent += module.getCurrent(CurrentUnit.AMPS);
        }
        totalCharge += (totalCurrent - MAX_CURRENT) * loopTime.seconds();
        totalCharge = Range.clip(totalCharge, 0, Double.POSITIVE_INFINITY);
        loopTime.reset();
        for (Module module : modules) {
            module.update();
            module.setDebugMode(isDebugMode);
        }
        if (!Thread.currentThread().isInterrupted()) {
            Context.robotPose = driveTrain.getPoseEstimate();
            Context.poseVelocity = Objects.requireNonNull(driveTrain.getPoseVelocity());
        }
        Context.packet.put("Loop Time", loopTime.milliseconds());
        Context.packet.put("Total Current", totalCurrent);
        Context.packet.put("Total Charge", totalCharge);
        Context.packet.put("MAX POWER", MAX_CHARGE);
        Context.packet.put("MID POWER", MID_CHARGE);
        Context.packet.put("MAX CURRENT", MAX_CURRENT);
    }

    /**
     * @return Whether the robot's current state is potentially hazardous to operate in
     */
    public boolean isHazardous() {
        for (Module module : modules) {
            if (module.isHazardous()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @return Whether the robot is currently doing work
     */
    public boolean isDoingWork() {
        for (Module module : modules) {
            if (module.isBusy()) {
                return true;
            }
        }
        return false;
    }
}