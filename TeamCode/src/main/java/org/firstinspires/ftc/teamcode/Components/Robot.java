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
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Robot {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AUw51u3/////AAABmS2SebfPGERUmfixnpbS89g79T2cQLWzcEcMv6u+RTGzrrvHwTVug45aIF3UiYJXKVzy/zhBFDleEJD2gEjPWWDQeYDV9k3htKwbHofAiOwRfivq8h2ZJIGcmUwiNT40UAEeUvQlKZXTcIYTrxiYmN4tAKEjmH5zKoAUfLefScv9gDDMwTYCKVm1M45M2a1VdIu0pMdoaJKo2DRZ3B+D+yZurFO/ymNtyAWME+1eE9PWyulZUmuUw/sDphp13KrdNHNbDUXwbunQN7voVm2HE5fWrFNtX5euVaPy/jedXTiM5KBeosXuemMeppimcTLHFvyhSwOMZMRhPT1Gus487FRWMt479sn2EhexfDCcd0JG";
    public static Dice dice = Dice.one;
    public static OpModeType opModeType = OpModeType.auto;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public OpenCvCamera webcam;

    private InterpLUT velocityController;
    private InterpLUT sleepController;
    private InterpLUT xController;
    private InterpLUT yController;

    public DcMotor intakeR, intakeL;

    public CRServo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public Servo rightIntakeHolder, leftIntakeHolder;

    public static Vector2d goal = new Vector2d(70.5275, -32.9725);
    public static Pose2d shootingPose = new Pose2d(-13.4725, -56.4725, Math.toRadians(15));
    public static Pose2d shootingPoseTele = new Pose2d(-4.4725, -36.4725, 0);

    public static Vector2d[] pwrShotLocals = new Vector2d[3];
    public static Vector2d[] pwrShots = new Vector2d[3];

    public static Vector2d A = new Vector2d(-5.4725, -50.4725);
    public static Vector2d B = new Vector2d(16.7775, -35.4725);
    public static Vector2d C = new Vector2d(47.5275, -53.4725);

    public static Pose2d robotPose = new Pose2d();
    public static Vector2d rightWobble = new Vector2d(-35.4725, -47.9725);

    public Launcher launcher;

    HardwareMap map;
    public SampleMecanumDrive driveTrain;

    public Robot(HardwareMap imported, double x, double y, double robotOrientation) {
        robotPose = new Pose2d(x - 70.4725, y - 70.4725, robotOrientation);
        construct(imported);
    }
    public Robot(HardwareMap imported, double x, double y, double robotOrientation, OpModeType type) {
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
        xController = new InterpLUT();
        yController = new InterpLUT();
        setVelocityController();
        setXController();
        setYController();
        pwrShots[0] = new Vector2d(70.4725, -1.0725);
        pwrShots[1] = new Vector2d(70.4725, -10.4725);
        pwrShots[2] = new Vector2d(70.4725, -19.4725);
        pwrShotLocals[0] = new Vector2d(-2.4725, -1.0725);
        pwrShotLocals[1] = new Vector2d(-2.4725, -10.4725);
        pwrShotLocals[2] = new Vector2d(-2.4725, -19.4725);
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
        velocityController.add(75,1500);
        velocityController.add(77.5,1320);
        velocityController.add(80,1340);
        //modified
        velocityController.add(85,1180);
        velocityController.add(90, 1180);
        velocityController.add(95,1170);
        //modified
        velocityController.add(100,1200);
        velocityController.add(105,1200);
        velocityController.add(110,1190);
        velocityController.add(115,1190);
        velocityController.add(120,1190);
        velocityController.add(125,1210);
        velocityController.add(132.5,1220);
        velocityController.add(1000,1000);
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
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(() -> {
            /*
             * Tell the webcam to start streaming images to us! Note that you must make sure
             * the resolution you specify is supported by the camera. If it is not, an exception
             * will be thrown.
             *
             * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
             * supports streaming from the webcam in the uncompressed YUV image format. This means
             * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
             * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
             *
             * Also, we specify the rotation that the webcam is used in. This is so that the image
             * from the camera sensor can be rotated such that it is always displayed with the image upright.
             * For a front facing camera, rotation is defined assuming the user is looking at the screen.
             * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
             * away from the user.
             */
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        });
//        initVuforia();
//        initTfod();
//        if(tfod != null) {
//            tfod.activate();
//        }
    }
    public double height = 0;
    public void scan(){
//        if(tfod != null) {
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                for (Recognition recognition : updatedRecognitions) {
//                    if (recognition.getHeight() < 200) {
//                        if (recognition.getHeight() >= 90) {
//                            dice = Dice.one;
//                        } else if (recognition.getHeight() < 90 && recognition.getHeight() > 10) {
//                            dice = Dice.two;
//                        } else {
//                            dice = Dice.three;
//                        }
//                        height = recognition.getHeight();
//                    }
//                }
//            }
//        }
    }
    public void turnOffVision(){
        if(tfod != null) {
            tfod.shutdown();
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
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}