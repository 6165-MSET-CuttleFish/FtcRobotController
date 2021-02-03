package org.firstinspires.ftc.teamcode.Components;

import android.telecom.Call;

import java.lang.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import java.util.List;
import java.util.concurrent.Callable;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Robot {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AUw51u3/////AAABmS2SebfPGERUmfixnpbS89g79T2cQLWzcEcMv6u+RTGzrrvHwTVug45aIF3UiYJXKVzy/zhBFDleEJD2gEjPWWDQeYDV9k3htKwbHofAiOwRfivq8h2ZJIGcmUwiNT40UAEeUvQlKZXTcIYTrxiYmN4tAKEjmH5zKoAUfLefScv9gDDMwTYCKVm1M45M2a1VdIu0pMdoaJKo2DRZ3B+D+yZurFO/ymNtyAWME+1eE9PWyulZUmuUw/sDphp13KrdNHNbDUXwbunQN7voVm2HE5fWrFNtX5euVaPy/jedXTiM5KBeosXuemMeppimcTLHFvyhSwOMZMRhPT1Gus487FRWMt479sn2EhexfDCcd0JG";
    private double robotLength;
    private double robotWidth;
    public int discs;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static OdometryGlobalCoordinatePosition position;

    public DcMotor topLeft;
    public DcMotor topRight;
    public DcMotor botLeft;
    public DcMotor botRight;

    public DcMotor intakeR, intakeL;

    public Servo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2;
    public Servo rightIntakeHolder, leftIntakeHolder;

    public static Goal hiGoal = new Goal(144, 37.5, 35.5);//142.75
    public static Goal loGoal = new Goal(144, 37.5, 17);
    public static Goal[] pwrShots = new Goal[3];

    public static Coordinate[] pwrShotLocals = new Coordinate[3];

    public static Coordinate A = new Coordinate(65, 20);
    public static Coordinate B = new Coordinate(78, 38);
    public static Coordinate C = new Coordinate(112, 5);
    public static Coordinate newA = new Coordinate(80, 30);
    public static Coordinate newB = new Coordinate(95, 55);
    public static Coordinate newC = new Coordinate(120, 20);

    public static Coordinate leftWobble = new Coordinate(45, 55);
    public static Coordinate rightWobble = new Coordinate(14, 24);

    public Launcher launcher;
    public PIDController pidRotate;
    double globalAngle, lastAngles;

    DcMotor.RunMode newRun;
    HardwareMap map;
    Callable<Boolean> overrides;

    public Robot(DcMotor.RunMode runMode, HardwareMap imported, double x, double y, double robotOrientation, double robotLength, double robotWidth, Callable<Boolean> overrides) {
        this.overrides = overrides;
        construct(runMode, imported, robotLength, robotWidth);
        position  = new OdometryGlobalCoordinatePosition(botLeft, botRight, topRight, 3072, 75, x, y, robotOrientation);
    }
    private void construct(DcMotor.RunMode runMode, HardwareMap imported, double robotLength, double robotWidth){
        pidRotate = new PIDController(.07, 0.014, 0.0044);
        //pidRotate = new PIDController(.00, .0000, 0);
//        pwrShots[0] = new Goal(144, 65.25, 23.5);
//        pwrShots[1] = new Goal(144, 60, 23.5);
//        pwrShots[2] = new Goal(144, 53.25, 23.5);
        pwrShots[0] = new Goal(144, 67, 23.5);
        pwrShots[1] = new Goal(144, 60, 23.5);
        pwrShots[2] = new Goal(144, 52.4, 23.5);

        pwrShotLocals[0] = new Coordinate(75, 67);
        pwrShotLocals[1] = new Coordinate(75, 60);
        pwrShotLocals[2] = new Coordinate(75, 53.25);
        this.robotWidth = robotWidth;
        this.robotLength = robotLength;
        map = imported;
        newRun = runMode;
        topLeft = map.dcMotor.get("fl");
        topRight = map.dcMotor.get("fr");
        botLeft = map.dcMotor.get("bl");
        botRight = map.dcMotor.get("br");
        intakeR = map.get(DcMotor.class, "intakeR");
        intakeL = map.get(DcMotor.class, "intakeL");
        in1 = map.get(Servo.class, "in1");
        in2 = map.get(Servo.class, "in2");


        arm1 = map.get(Servo.class, "wobbleArm1");
        arm2 = map.get(Servo.class, "wobbleArm2");
        //arm1.setDirection( Servo.Direction.REVERSE);
//        arm1.setPosition(0.93);
//        arm2.setPosition(0.07);
        grabber = map.get(Servo.class, "wobbleGrabber1");
        grabber2 = map.get(Servo.class, "wobbleGrabber2");
        //grabber.setPosition(0.92);

        leftIntakeHolder = map.get(Servo.class,"wallL");
        rightIntakeHolder = map.get(Servo.class,"wallR");
//        lockIntake();
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher = new Launcher(map);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(newRun);
        topRight.setMode(newRun);
        botLeft.setMode(newRun);
        botRight.setMode(newRun);


        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        botLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        botRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public Robot(DcMotor.RunMode runMode, HardwareMap imported, double robotLength, double robotWidth, Callable<Boolean> overrides) {
        this.overrides = overrides;
        construct(runMode, imported, robotLength, robotWidth);
        if(position == null){
            position  = new OdometryGlobalCoordinatePosition(botLeft, botRight, topRight, 3072, 75, 0, 0, 0);
        }
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed) {
        try {
            goTo(pt, power, preferredAngle, turnSpeed, () -> {});
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed, Runnable block) throws Exception {
        double distance = Math.hypot(pt.x - position.getX(), pt.y - position.y);
        while(distance > 3.5 && overrides.call()) {
            distance = Math.hypot(pt.x - position.x, pt.y - position.y);
            if(distance < 20){
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
            double rx = turnSpeed*Range.clip((AngleWrap(preferredAngle - position.radians()))/Math.toRadians(20), -1, 1);
            //double movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
            setMovement(movement_x, movement_y, -rx);
        }
        setMovement(0, 0, 0);
    }
    public void wingsOut() {
        leftIntakeHolder.setPosition(.23);
        rightIntakeHolder.setPosition(.84);
    }
    public void wingsIn() {
        leftIntakeHolder.setPosition(0.91);
        rightIntakeHolder.setPosition(0.18);
    }
    public void unlockIntake(){
        rightIntakeHolder.setPosition(0.4);
    }
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    public void setMovement(double lx, double ly, double rx){
        topLeft.setPower(Range.clip(ly + lx + rx, -1, 1));
        topRight.setPower(Range.clip(ly - lx - rx, -1, 1));
        botLeft.setPower(Range.clip(ly - lx + rx, -1, 1));
        botRight.setPower(Range.clip(ly + lx - rx, -1, 1));
    }
    public void init(){
       Thread newThread = new Thread(position);
       newThread.start();
    }
    public void autoInit(){
        init();
        initVuforia();
        initTfod();
        if(tfod != null) {
            tfod.activate();
        }
    }
    public double height = -1;
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
                            discs = 4;
                        } else if (recognition.getHeight() < 90 && recognition.getHeight() > 10) {
                            discs = 1;
                        } else {
                            discs = 0;
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
            in1.setPosition(1);
            in2.setPosition(0);
        }
        else if(intakeSpeed<0){
            in1.setPosition(0);
            in2.setPosition(1);
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
    public void launcherturnTo(Coordinate pt, double pwr){
        Coordinate shooter = position.toPoint();
        shooter.polarAdd(position.radians() + Math.PI/2, 15);
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
        pidRotate.setTolerance(0.9);
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