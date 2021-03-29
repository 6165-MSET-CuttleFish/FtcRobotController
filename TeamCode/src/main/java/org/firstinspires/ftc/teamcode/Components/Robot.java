package org.firstinspires.ftc.teamcode.Components;

import java.lang.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    public Dice dice = Dice.one;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static OdometryGlobalCoordinatePosition position;

//    public DcMotor topLeft;
//    public DcMotor topRight;
//    public DcMotor botLeft;
//    public DcMotor botRight;

    public DcMotor intakeR, intakeL;

    public Servo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public Servo rightIntakeHolder, leftIntakeHolder;

    public static Goal hiGoal = new Goal(141, 37.5, 35.5);//142.75
    public static Goal loGoal = new Goal(141, 37.5, 17);
    public static Goal[] pwrShots = new Goal[3];

    public static Vector2d[] pwrShotLocals = new Vector2d[3];

    public static Pose2d A = new Pose2d(65, 20);
    public static Pose2d B = new Pose2d(87.25, 35);
    public static Pose2d C = new Pose2d(108, 9);
    public static Pose2d newA = new Pose2d(80.25, 25);
    public static Pose2d newB = new Pose2d(106.25, 52);
    public static Pose2d newC = new Pose2d(120, 27);

    public static Pose2d robotPose;
    public Pose2d startPose;

    public static Coordinate leftWobble = new Coordinate(34.7, 53);
    public static Coordinate rightWobble = new Coordinate(14, 24);

    public Launcher launcher;
    public PIDController pidRotate;
    double globalAngle, lastAngles;

    HardwareMap map;
    Callable<Boolean> overrides;
    Telemetry telemetry;
    public SampleMecanumDrive driveTrain;

    public Robot(HardwareMap imported, double x, double y, double robotOrientation, Telemetry telemetry, Callable<Boolean> overrides) {
        this.overrides = overrides;
        construct(imported, telemetry);
        startPose = new Pose2d(x, y, robotOrientation);
        driveTrain.setPoseEstimate(startPose);
       // position  = new OdometryGlobalCoordinatePosition(botLeft, botRight, topRight, 8192/(1.5*Math.PI), 3, x, y, robotOrientation);
    }
    private void construct(HardwareMap imported, Telemetry telemetry){
        driveTrain = new SampleMecanumDrive(imported);
        pidRotate = new PIDController(.07, 0.013, 0.004);
        this.telemetry = telemetry;
        pwrShots[0] = new Goal(142, 70.5, 23.5);
        pwrShots[1] = new Goal(142, 59, 23.5);
        pwrShots[2] = new Goal(142, 53.25, 23.5);

        pwrShotLocals[0] = new Vector2d(67, 67);
        pwrShotLocals[1] = new Vector2d(67, 60);
        pwrShotLocals[2] = new Vector2d(67, 53.25);
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
    }
    public Robot(HardwareMap imported, Telemetry telemetry, Callable<Boolean> overrides) {
        this.overrides = overrides;
        construct(imported, telemetry);
//        if(position == null){
//            //position  = new OdometryGlobalCoordinatePosition(intakeL, launcher.flywheel1, intakeR, 8192/(1.5*Math.PI), 3, 0, 0, 0);
//        }
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed) {
        goTo(pt, power, preferredAngle, turnSpeed, () -> {});
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed, Runnable block) {
        try {
            double distance = Math.hypot(pt.x - position.getX(), pt.y - position.y);
            while (distance > 3.5 && overrides.call()) {
                distance = Math.hypot(pt.x - position.x, pt.y - position.y);
                if (distance < 15) {
                    block.run();
                }
                double absAngleToTarget = Math.atan2(pt.y - position.y, pt.x - position.x);

                double relAngleToPoint = AngleWrap(absAngleToTarget - position.radians() + Math.toRadians(90));
                //System.out.println("Rel " + relAngleToPoint);
                double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
                double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
                double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
                double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

                double movement_x = movementXPower * power;
                double movement_y = movementYPower * power;
                double relTurnAngle = relAngleToPoint - Math.toRadians(90) + preferredAngle;
                double movement_turn = distance > 5 ? Range.clip(relTurnAngle / Math.toRadians(20), -1, 1) * turnSpeed : 0;
                double rx = turnSpeed * Range.clip((AngleWrap(preferredAngle - position.radians())) / Math.toRadians(20), -1, 1);
                //double movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
                setMovement(movement_x, movement_y, -rx);
            }
            setMovement(0, 0, 0);
        } catch (Exception e){
            e.printStackTrace();
        }
    }
    public void knockPowerShots(){
        Trajectory trajectory = driveTrain.trajectoryBuilder(startPose)
                .splineTo(pwrShotLocals[2], 0)
                .build();
        driveTrain.followTrajectory(trajectory);
        launcher.singleRound();
        Trajectory newTraj = driveTrain.trajectoryBuilder(trajectory.end())
                .splineTo(pwrShotLocals[1], 0)
                .build();
        driveTrain.followTrajectory(newTraj);
        launcher.singleRound();
        Trajectory newTraj1 = driveTrain.trajectoryBuilder(newTraj.end())
                .splineTo(pwrShotLocals[0], 0)
                .build();
        driveTrain.followTrajectory(newTraj1);
        launcher.singleRound();
    }
    public Trajectory dropZone(){
        switch(dice){
            case one: return driveTrain.trajectoryBuilder().splineToSplineHeading(A, Math.toRadians(-180)).build();
            case two: return driveTrain.trajectoryBuilder().splineToSplineHeading(B, Math.toRadians(-180)).build();
            default: return driveTrain.trajectoryBuilder().splineToSplineHeading(C, Math.toRadians(-180)).build();
        }
    }
//    public static Trajectory shootingPath = driveTrain.trajectoryBuilder()
//            .splineToSplineHeading(new Pose2d(57, hiGoal.y),0)
//            .build();
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
    public void setMovement(double lx, double ly, double rx){
//        topLeft.setPower(Range.clip(ly + lx + rx, -1, 1));
//        topRight.setPower(Range.clip(ly - lx - rx, -1, 1));
//        botLeft.setPower(Range.clip(ly - lx + rx, -1, 1));
//        botRight.setPower(Range.clip(ly + lx - rx, -1, 1));
//        telemetry.addData("x", position.x);
//        telemetry.addData("y", position.y);
//        telemetry.addData("orientation(degrees)", position.returnOrientation());
//        telemetry.update();
    }
    public void init(){
        Thread launcherThread = new Thread(launcher);
        launcherThread.start();
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
//                    if (recognition.getLabel() == LABEL_FIRST_ELEMENT) {
//                        discs = 4;
//                    } else if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
//                        discs = 1;
//                    } else {
//                        discs = 0;
//                    }
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
        if(dice == Dice.one){
            leftWobble.addY(-5);
        }
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
    public double getHeading() {//for overall
        return position.returnOrientation();
    }
    public void resetAngle()
    {
        lastAngles =  position.returnOrientation();

        globalAngle = 0;
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
    public double getAngle()
    {
        double angles = position.returnOrientation();

        double deltaAngle = angles - lastAngles;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void orient(double angle, double pwr) {
        double closest = angle - position.returnOrientation();
        try {
            pidRotate(closest, pwr);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void turnTo(Coordinate pt, double pwr){
        double absAngleToTarget = Math.atan2(pt.y - position.y, pt.x - position.x);
        double relAngleToPoint = AngleWrap(absAngleToTarget - position.radians());
        //orient(position.angleTo(pt), pwr);
        try {
            pidRotate(Math.toDegrees(relAngleToPoint), pwr);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void launcherTurnTo(Coordinate pt, double pwr){
        Coordinate shooter = position.toPoint();
        shooter.polarAdd(position.radians() + Math.PI/2, 8);
        double absAngleToTarget = Math.atan2(pt.y - shooter.y, pt.x - shooter.x);
        double relAngleToPoint = AngleWrap(absAngleToTarget - position.radians());
        try {
            pidRotate(Math.toDegrees(relAngleToPoint), pwr);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void pidRotate(double degrees, double power) throws Exception{
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0.15, power);
        pidRotate.setTolerance(1.4);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0 && overrides.call())
            {
                setMovement(0, 0 , power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setMovement(0, 0, -power);
            } while (!pidRotate.onTarget() && overrides.call());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setMovement(0, 0, -power);
            } while (!pidRotate.onTarget() && overrides.call());

        // turn the motors off.
        setMovement(0, 0 ,0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}