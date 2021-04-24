package org.firstinspires.ftc.teamcode.Components;

import java.lang.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AUw51u3/////AAABmS2SebfPGERUmfixnpbS89g79T2cQLWzcEcMv6u+RTGzrrvHwTVug45aIF3UiYJXKVzy/zhBFDleEJD2gEjPWWDQeYDV9k3htKwbHofAiOwRfivq8h2ZJIGcmUwiNT40UAEeUvQlKZXTcIYTrxiYmN4tAKEjmH5zKoAUfLefScv9gDDMwTYCKVm1M45M2a1VdIu0pMdoaJKo2DRZ3B+D+yZurFO/ymNtyAWME+1eE9PWyulZUmuUw/sDphp13KrdNHNbDUXwbunQN7voVm2HE5fWrFNtX5euVaPy/jedXTiM5KBeosXuemMeppimcTLHFvyhSwOMZMRhPT1Gus487FRWMt479sn2EhexfDCcd0JG";
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final boolean DEBUG = true; // if debug is wanted, change to true
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    public static UGContourRingPipeline.Height height = UGContourRingPipeline.Height.ZERO;
    public static OpModeType opModeType = OpModeType.none;

    private LinearOpMode linearOpMode;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public OpenCvCamera webcam;
    private UGContourRingPipeline pipeline;
    private CustomPipeline bouncebacks;

    private InterpLUT velocityController;
    private InterpLUT sleepController;

    public DcMotor intakeR, intakeL;

    public CRServo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public Servo rightIntakeHolder, leftIntakeHolder;

    public static Vector2d goal = new Vector2d(70.5275, -32.9725);
    public static Pose2d shootingPose = new Pose2d(-14.4725, -52.5, Math.toRadians(2));
    public static Pose2d shootingPoseTele = new Pose2d(-5, -32.9725, Math.toRadians(-3));

    public static Vector2d[] pwrShotLocals = new Vector2d[3];
    public static Vector2d[] pwrShots = new Vector2d[3];

    public static Vector2d A = new Vector2d(-5.4725, -50.4725);
    public static Vector2d B = new Vector2d(16.7775, -35.4725);
    public static Vector2d C = new Vector2d(45.5275, -57.4);

    public static Pose2d robotPose = new Pose2d();
    public static Vector2d rightWobble = new Vector2d(-40, -42.3);

    public Launcher launcher;

    HardwareMap map;
    public SampleMecanumDrive driveTrain;

    public Robot(HardwareMap imported, double x, double y, double robotOrientation) {
        robotPose = new Pose2d(x - 70.4725, y - 70.4725, robotOrientation);
        construct(imported);
    }
    public Robot(HardwareMap imported, double x, double y, double robotOrientation, OpModeType type, LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        opModeType = type;
        robotPose = new Pose2d(x - 70.4725, y - 70.4725, robotOrientation);
        construct(imported);
    }
    public Robot(HardwareMap imported, OpModeType type){
        opModeType = type;
        construct(imported);
    }
    private void construct(HardwareMap imported){
        velocityController = new InterpLUT();
        sleepController = new InterpLUT();
        setVelocityController();
        setSleepController();
        pwrShots[0] = new Vector2d(70.4725, -1.4725);
        pwrShots[1] = new Vector2d(70.4725, -10.4725);
        pwrShots[2] = new Vector2d(70.4725, -19.4725);
        pwrShotLocals[0] = new Vector2d(-6.03, -6.4);
        pwrShotLocals[1] = new Vector2d(-6.03, -14.5);
        pwrShotLocals[2] = new Vector2d(-6.03, -22.7);
        map = imported;
        int cameraMonitorViewId = this
                .map
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        map.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(map.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        intakeR = map.get(DcMotor.class, "intakeR");
        intakeL = map.get(DcMotor.class, "intakeL");
        in1 = map.crservo.get("in1");
        in2 = map.crservo.get("in2");
        in1.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1 = map.get(Servo.class, "wobbleArm1");
        arm2 = map.get(Servo.class, "wobbleArm2");
        grabber = map.get(Servo.class, "wobbleGrabber1");
        grabber2 = map.get(Servo.class, "wobbleGrabber2");
        leftIntakeHolder = map.get(Servo.class,"wallL");
        rightIntakeHolder = map.get(Servo.class,"wallR");
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher = new Launcher(map);
        driveTrain = new SampleMecanumDrive(imported, linearOpMode);
        driveTrain.setPoseEstimate(robotPose);
        driveTrain.setVelocityCallable(()->launcher.getVelocity());
        driveTrain.setTargetVeloCallable(()->launcher.getTargetVelo());
    }
    private void setVelocityController(){
        velocityController.add(0,1580);
        velocityController.add(75,1580);
        velocityController.add(77.5,1580);
        velocityController.add(80,1500);
        velocityController.add(85,1435);
        velocityController.add(90, 1285);
        //tbc
        velocityController.add(95,1250);
        velocityController.add(100,1250);
        velocityController.add(105,1220);
        velocityController.add(110,1190);
        velocityController.add(115,1190);
        velocityController.add(120,1190);
        velocityController.add(125,1210);
        velocityController.add(132.5,1220);
        velocityController.add(1000,1000);
        velocityController.createLUT();
    }
    private void setSleepController(){
        sleepController.add(0, 80);
        sleepController.add(75, 80);
        sleepController.add(77.5, 80);
        sleepController.add(80, 80);
        sleepController.add(85, 80);
        sleepController.add(90, 90);
        sleepController.add(95, 100);
        sleepController.add(100, 110);
        sleepController.add(105, 130);
        sleepController.add(2000, 150);
        sleepController.createLUT();
    }
    public double getPoseVelo(Pose2d pose2d){
        return  velocityController.get(Range.clip(Coordinate.distanceToLine(pose2d, goal.getX()), 0, 140));
        //return velocityController.get(pose2d.vec().distTo(goal));
    }
    public double getPoseVelo(Vector2d vec){
        return velocityController.get(vec.distTo(goal));
    }
    public Robot(HardwareMap imported) {
        robotPose = new Pose2d();
        construct(imported);
    }
    public void unlockIntake(){
        launcher.unlockIntake();
    }
    public void init(){
    }
    public void autoInit(){
        init();
        webcam.setPipeline(pipeline = new UGContourRingPipeline(linearOpMode.telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        CustomPipeline.CAMERA_WIDTH = CAMERA_WIDTH;

        CustomPipeline.HORIZON = HORIZON;

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }
    public void switchPipeline(){
        webcam.setPipeline(bouncebacks = new CustomPipeline(linearOpMode));
    }
    public void scan(){
        height = pipeline.getHeight();
    }
    public void turnOffVision(){
        //webcam.closeCameraDeviceAsync(()-> webcam.stopStreaming());
    }
    public void wobbleArmUp() {
        arm1.setPosition(0.1);
        arm2.setPosition (0.88);
    }
    public void wobbleArmDown() {
        arm1.setPosition(0.93);
        arm2.setPosition (0.07);

    }
    public void wobbleArmVertical(){
        arm1.setPosition(0.5);
        arm2.setPosition (0.5);
    }
    public void grab(){
        grabber.setPosition(0.13);
        grabber2.setPosition(0.83);
    }
    public void release(){
        grabber.setPosition(0.63);
        grabber2.setPosition(0.29);
    }
    public void intake(double intakeSpeed){
        intakeL.setPower(-intakeSpeed);
        intakeR.setPower(-intakeSpeed);
        in1.setPower(intakeSpeed);
        in2.setPower(intakeSpeed);
    }
    public void optimalShoot(int rounds){
        launcher.customShoot(sleepController.get(driveTrain.getPoseEstimate().vec().distTo(goal)), rounds);
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public Trajectory ringPickup(){
        TrajectoryBuilder builder = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate());
        builder = builder
                .addTemporalMarker(0.3, () -> {
                    launcher.wingsVert();
                    Async.set(() -> Robot.C.distTo(driveTrain.getPoseEstimate().vec()) <= 11, this::wobbleArmDown);
                });
        ArrayList<Vector2d> initialRings = new ArrayList<>();
        ArrayList<Vector2d> enRouteRings = new ArrayList<>();
        for(Vector2d wayPoint : bouncebacks.getVectors()){
            if(wayPoint.getX() < 40){
                initialRings.add(wayPoint);
            } else if(wayPoint.getX() >= 40){
                enRouteRings.add(wayPoint);
            }
        }
        for(Vector2d wayPoint : initialRings){
            builder = builder.splineTo(wayPoint, 0);
        }
        for(Vector2d wayPoint : enRouteRings){
            if(wayPoint.getX() < 57) builder = builder.splineTo(wayPoint, Math.toRadians(-90));
            else builder = builder.splineToSplineHeading(Coordinate.toPose(wayPoint, Math.toRadians(-85)), Math.toRadians(-90));
        }
        builder = builder
                .splineToSplineHeading(Coordinate.toPose(getWayPoint(), Math.toRadians(-190)), Math.toRadians(-190))
                .addDisplacementMarker(() -> {
                    Async.start(() -> {
                        release();
                        sleep(400);
                        wobbleArmUp();
                    });
                    launcher.setVelocity(getPoseVelo(shootingPose));
                    launcher.flapDown();
                    launcher.safeLeftOut();
                });
        return builder.build();
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public Vector2d getWayPoint(){
        if(height == UGContourRingPipeline.Height.FOUR){
            return C;
        } else if(height == UGContourRingPipeline.Height.ONE){
            return B;
        } else {
            return A;
        }
    }
}