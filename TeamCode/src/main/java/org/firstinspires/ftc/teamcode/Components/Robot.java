package org.firstinspires.ftc.teamcode.Components;

import java.lang.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import java.util.List;
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
    private int discs;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static OdometryGlobalCoordinatePosition position;
    public DcMotor topLeft;
    public DcMotor topRight;
    public DcMotor botLeft;
    public DcMotor botRight;

    public Launcher launcher;
    Orientation orientation = new Orientation();
    Orientation lastAngles = new Orientation();
    PIDController pidRotate, pidDrive, pidStrafe, pidCurve, pidCorrection;
    DcMotor.RunMode newRun;
    HardwareMap map;


    public boolean auto;
    public double ninja = 1;
    public Robot(DcMotor.RunMode runMode, HardwareMap imported, double x, double y, double robotLength, double robotWidth) {
        auto = true;
        map = imported;
        newRun = runMode;
        topLeft = map.dcMotor.get("topLeft");
        topRight = map.dcMotor.get("topRight");
        botLeft = map.dcMotor.get("botLeft");
        botRight = map.dcMotor.get("botRight");


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(newRun);//set to imported runMode
        topRight.setMode(newRun);
        botLeft.setMode(newRun);
        botRight.setMode(newRun);


        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);//in reverse because of motor direction
        botLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //bot = new OdometryGlobalCoordinatePosition(topLeft, topRight, botLeft, 560, 760, x, y);
        position  = new OdometryGlobalCoordinatePosition(topLeft, topRight, botLeft, 307.699557, 760, x, y);

    }
    public static int index = 0;
    public void goTo(Coordinate pt, double power, double preferredAngle, double turnSpeed){
        double distance = Math.hypot(pt.x - position.getX(), pt.y - position.y);
        if(distance < 5) {
            setMovement(0, 0, 0);
            index ++;
            return;
        }
        distance = Math.hypot(pt.x - position.x, pt.y - position.y);
        //double absAngleToTarget = Math.atan2(y - worldAngle_rad, x - worldAngle_rad);
        //double absAngleToTarget = Math.atan2(y - worldAngle_rad, x - worldAngle_rad);
        double absAngleToTarget = Math.atan2(pt.y - position.y, pt.x - position.x);
        //System.out.println("Abs " + absAngleToTarget);
        double relAngleToPoint = AngleWrap(absAngleToTarget - position.robotOrientationRadians + Math.toRadians(90));
        //System.out.println("Rel " + relAngleToPoint);
        double relativeXToPoint = Math.cos(relAngleToPoint) * distance;
        double relativeYToPoint = Math.sin(relAngleToPoint) * distance;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
//        double relativeTurnAngle = relAngleToPoint - Math.toRadians(180) + preferredAngle;
//        double movementTurn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)*turnSpeed;

        double movement_x = movementXPower * power;
        double movement_y = movementYPower * power;
        double relTurnAngle = relAngleToPoint - Math.toRadians(90) + preferredAngle;
        double movement_turn = distance > 10 ? Range.clip(relTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed : 0;
        setMovement(movement_x, movement_y, movement_turn);
    }
    public void setMovement(double lx, double ly, double rx){
        topLeft.setPower(Range.clip(ninja * ly + lx + rx, -1, 1));//1.1   ; -.9
        topRight.setPower(Range.clip(ninja * ly - lx - rx, -1, 1));//-1.1  ; .9
        botLeft.setPower(Range.clip(ninja * ly - lx + rx, -1, 1));//-.9   ; 1.1
        botRight.setPower(Range.clip(ninja * ly + lx - rx, -1, 1));//.9  ; -1.1

    }
    public void init(){
       Thread newThread = new Thread(position);
       newThread.start();
    }
    public void autoInit(){
        init();
        initVuforia();
        initTfod();
        tfod.activate();
    }
    public void scan(){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if(recognition.getLabel() == LABEL_FIRST_ELEMENT){
                    discs = 4;
                }
                else if(recognition.getLabel() == LABEL_SECOND_ELEMENT){
                    discs = 1;
                }
                else{
                    discs = 0;
                }
            }
        }
    }
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void aimAt(Coordinate target){
        launcher.findAngle(position.x, position.y, target.x, target.y);
    }
    public double getHeading() {//for overall
        return position.returnOrientation();
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
    enum heading{
        forward,
        backward,
        left,
        right
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
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}