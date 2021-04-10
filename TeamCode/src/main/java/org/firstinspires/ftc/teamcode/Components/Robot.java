package org.firstinspires.ftc.teamcode.Components;

import java.lang.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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

    public CRServo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public Servo rightIntakeHolder, leftIntakeHolder;

    public static Vector2d goal = new Vector2d(141, 37.5);
    public static Pose2d shootingPose = new Pose2d(57, 20, Math.toRadians(0));

    public static Vector2d[] pwrShotLocals = new Vector2d[3];

    public static Vector2d A = new Vector2d(65, 20);
    public static Vector2d B = new Vector2d(87.25, 35);
    public static Vector2d C = new Vector2d(120, 13);
    public static Vector2d newA = new Vector2d(80.25, 25);
    public static Vector2d newB = new Vector2d(106.25, 52);
    public static Vector2d newC = new Vector2d(120, 27);

    public static Pose2d robotPose = new Pose2d();

    public static Vector2d leftWobble = new Vector2d(34.7, 53);
    public static Vector2d rightWobble = new Vector2d(25, 33.5);

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

        pwrShotLocals[0] = new Vector2d(68, 69.4);
        pwrShotLocals[1] = new Vector2d(68, 60);
        pwrShotLocals[2] = new Vector2d(68, 51);
        map = imported;
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
        driveTrain = new SampleMecanumDrive(imported);
        driveTrain.setPoseEstimate(robotPose);
        driveTrain.setVelocityCallable(()->launcher.getVelocity());
        driveTrain.setTargetVeloCallable(()->launcher.getTargetVelo());
    }
    private void setVelocityController(){
        velocityController.add(0,1700);
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
        velocityController.add(1000,0);
        velocityController.createLUT();
    }
    private void setXController(){
    }
    private void setYController(){
    }
    public double getPoseVelo(Pose2d pose2d){
        return Range.clip(velocityController.get(pose2d.vec().distTo(goal)), 0, 1350);
    }
    public double getPoseVelo(Vector2d vec){
        return Range.clip(velocityController.get(vec.distTo(goal)), 0, 1350);
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
            in1.setPower(1);
            in2.setPower(1);
        }
        else if(intakeSpeed<0){
            in1.setPower(-1);
            in2.setPower(-1);
        }
        else{
            in1.setPower(0);
            in2.setPower(0);
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