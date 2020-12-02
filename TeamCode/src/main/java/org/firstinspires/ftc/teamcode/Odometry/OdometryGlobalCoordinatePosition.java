package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePosition extends Coordinate implements Runnable{
    //Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;
    private double COUNTS_PER_INCH;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay, double x, double y, double orientation){
        super(x, y);
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;
        this.robotOrientationRadians = 0;
        robotOrientationRadians = orientation;
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        this.COUNTS_PER_INCH = COUNTS_PER_INCH;
        reverseLeftEncoder();
        reverseRightEncoder();
        //reverseNormalEncoder();
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;
        //temp swap
        double dY = p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians);
        double dX = p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians);
        //Calculate and update the position values
        //robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        //robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
        add(1.81158182*dX/COUNTS_PER_INCH, -1.81158182*dY/COUNTS_PER_INCH);
        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
//    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }
//
//    /**
//     * Returns the robot's global y coordinate
//     * @return global y coordinate
//     */
//    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){
        double angle =  -Math.toDegrees(robotOrientationRadians);
        if(angle > 180){
            angle -= 360;
        }
        else if(angle < -180){
            angle += 360;
        }
        return angle;
    }
    public double radians(){ return -robotOrientationRadians; }
    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }
    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
