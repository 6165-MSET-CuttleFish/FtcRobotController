package org.firstinspires.ftc.teamcode.Components;

import java.lang.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import java.util.List;
import java.util.concurrent.Callable;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Robot {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AUw51u3/////AAABmS2SebfPGERUmfixnpbS89g79T2cQLWzcEcMv6u+RTGzrrvHwTVug45aIF3UiYJXKVzy/zhBFDleEJD2gEjPWWDQeYDV9k3htKwbHofAiOwRfivq8h2ZJIGcmUwiNT40UAEeUvQlKZXTcIYTrxiYmN4tAKEjmH5zKoAUfLefScv9gDDMwTYCKVm1M45M2a1VdIu0pMdoaJKo2DRZ3B+D+yZurFO/ymNtyAWME+1eE9PWyulZUmuUw/sDphp13KrdNHNbDUXwbunQN7voVm2HE5fWrFNtX5euVaPy/jedXTiM5KBeosXuemMeppimcTLHFvyhSwOMZMRhPT1Gus487FRWMt479sn2EhexfDCcd0JG";
    public static Dice dice = Dice.one;
    public static OpModeType opModeType = OpModeType.auto;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private InterpLUT velocityController;
    private InterpLUT xController;
    private InterpLUT yController;

    public DcMotor intakeR, intakeL;

    public Servo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public Servo rightIntakeHolder, leftIntakeHolder;

    public static Vector2d goal = new Vector2d(141, 37.5);
    public static Pose2d shootingPose = new Pose2d(55, 37.5, 0);

    public static Vector2d[] pwrShotLocals = new Vector2d[3];

    public static Vector2d A = new Vector2d(65, 20);
    public static Vector2d B = new Vector2d(87.25, 35);
    public static Vector2d C = new Vector2d(108, 9);
    public static Vector2d newA = new Vector2d(80.25, 25);
    public static Vector2d newB = new Vector2d(106.25, 52);
    public static Vector2d newC = new Vector2d(120, 27);

    public static Pose2d robotPose = new Pose2d();

    public static Vector2d leftWobble = new Vector2d(34.7, 53);
    public static Vector2d rightWobble = new Vector2d(14, 24);

    public Launcher launcher;

    HardwareMap map;
    public SampleMecanumDrive driveTrain;

    public Robot(HardwareMap imported, double x, double y, double robotOrientation) {
        robotPose = new Pose2d(x, y, robotOrientation);
        construct(imported);
    }
    public Robot(HardwareMap imported, double x, double y, double robotOrientation, OpModeType type) {
        opModeType = type;
        robotPose = new Pose2d(x, y, robotOrientation);
        construct(imported);
    }
    public Robot(HardwareMap imported, OpModeType type){
        opModeType = type;
        construct(imported);
    }
    private void construct(HardwareMap imported){
        velocityController = new InterpLUT();
        xController = new InterpLUT();
        yController = new InterpLUT();
        setVelocityController();
        setXController();
        setYController();

        pwrShotLocals[0] = new Vector2d(67, 67);
        pwrShotLocals[1] = new Vector2d(67, 58);
        pwrShotLocals[2] = new Vector2d(67, 50.25);
        map = imported;
        intakeR = map.get(DcMotor.class, "intakeR");
        intakeL = map.get(DcMotor.class, "intakeL");
        in1 = map.get(Servo.class, "in1");
        in2 = map.get(Servo.class, "in2");

        arm1 = map.get(Servo.class, "wobbleArm1");
        arm2 = map.get(Servo.class, "wobbleArm2");
        grabber = map.get(Servo.class, "wobbleGrabber1");
        grabber2 = map.get(Servo.class, "wobbleGrabber2");
        leftIntakeHolder = map.get(Servo.class,"wallL");
        rightIntakeHolder = map.get(Servo.class,"wallR");
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher = new Launcher(map);
        driveTrain = new SampleMecanumDrive(imported);
        driveTrain.setPoseEstimate(robotPose);
    }
    private void setVelocityController(){
        velocityController.add(75,1360);
        velocityController.add(77.5,1320);
        velocityController.add(80,1340);
        velocityController.add(85,1300);
        velocityController.add(90, 1260);
        velocityController.add(95,1220);
        velocityController.add(100,1200);
        velocityController.add(105,1200);
        velocityController.add(110,1190);
        velocityController.add(115,1190);
        velocityController.add(120,1190);
        velocityController.add(125,1210);
        velocityController.add(132.5,1220);
    }
    private void setXController(){
    }
    private void setYController(){
    }
    public double getPoseVelo(){
        return Range.clip(0, 1350, velocityController.get(driveTrain.getPoseEstimate().vec().distTo(goal)));
    }
    public Robot(HardwareMap imported) {
        construct(imported);
    }
    public void wingsOut() {
        launcher.wingsOut();
    }
    public void wingsIn() {
        launcher.wingsIn();
    }

    public void wingsMid() {
        launcher.wingsMid();
    }

    public void leftOut() {
        launcher.leftOut();
    }

    public void unlockIntake(){
        launcher.unlockIntake();
    }
    public void init(){
    }
    public void autoInit(){
        init();
        initVuforia();
        initTfod();
        if(tfod != null) {
            tfod.activate();
        }
    }
    public double height = 0;
    public void scan(){
        if(tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getHeight() < 200) {
                        if (recognition.getHeight() >= 90) {
                            dice = Dice.one;
                        } else if (recognition.getHeight() < 90 && recognition.getHeight() > 10) {
                            dice = Dice.two;
                        } else {
                            dice = Dice.three;
                        }
                        height = recognition.getHeight();
                    }
                }
            }
        }
    }
    public void turnOffVision(){
        if(tfod != null) {
            tfod.shutdown();
        }
    }
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public final void idle() {
        Thread.yield();
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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
        sleep(100);
    }
    public void release(){
        grabber.setPosition(0.63);
        grabber2.setPosition(0.29);
        sleep(100);
    }
    public void intake(double intakeSpeed){
        intakeL.setPower(-intakeSpeed);
        intakeR.setPower(-intakeSpeed);
        if(intakeSpeed>0){
            in1.setPosition(0);
            in2.setPosition(1);
        }
        else if(intakeSpeed<0){
            in1.setPosition(1);
            in2.setPosition(0);
        }
        else{
            in1.setPosition(0.5);
            in2.setPosition(0.5);
        }
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
}