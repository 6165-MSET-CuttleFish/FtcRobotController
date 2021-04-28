package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;

import java.util.ArrayList;

public class RingLocalizer extends OpenCvPipeline {
    public RingLocalizer(LinearOpMode opMode){
        ret = new Mat();
        mat = new Mat();
        this.linearOpMode = opMode;
        areaPerpendicular = new InterpLUT();
        angleCalculator = new InterpLUT();
        createControlPoints();
    }
    private void createControlPoints(){
        setAreaParser();
        setAngleRegression();
    }
    private void setAreaParser(){
        areaPerpendicular.add(0, 70);
        areaPerpendicular.add(500, 70);
        areaPerpendicular.add(630, 68);
        areaPerpendicular.add(660, 66);
        areaPerpendicular.add(720, 64);
        areaPerpendicular.add(730, 62);
        areaPerpendicular.add(736, 60);
        areaPerpendicular.add(816, 58);
        areaPerpendicular.add(840, 56);
        areaPerpendicular.add(864, 54);
        areaPerpendicular.add(936, 52);
        areaPerpendicular.add(988, 50);
        areaPerpendicular.add(1040, 48);
        areaPerpendicular.add(1176, 46);
        areaPerpendicular.add(1190, 44);
        areaPerpendicular.add(1276, 42);
        areaPerpendicular.add(1320, 40);
        areaPerpendicular.add(1380, 38);
        areaPerpendicular.add(1536, 36);
        areaPerpendicular.add(1716, 34);
        areaPerpendicular.add(1836, 32);
        areaPerpendicular.add(2072, 30);
        areaPerpendicular.add(2204, 28);
        areaPerpendicular.add(2460, 26);
        areaPerpendicular.add(2752, 24);
        areaPerpendicular.add(3102, 22);
        areaPerpendicular.add(3600, 20);
        areaPerpendicular.add(4180, 18);
        areaPerpendicular.add(4838, 16);
        areaPerpendicular.add(8000, 0);
        areaPerpendicular.createLUT();
    }
    private void setAngleRegression(){
        angleCalculator.add(0,10.88552705);
        angleCalculator.add(2,9.462322208);
        angleCalculator.add(8,8.746162263);
        angleCalculator.add(22,6.581944655);
        angleCalculator.add(42,3.667788056);
        angleCalculator.add(64,0.7345210343);
        angleCalculator.add(82,-2.202598162);
        angleCalculator.add(104,-5.128191042);
        angleCalculator.add(112,-8.02723751);
        angleCalculator.add(126,-10.88552705);
        angleCalculator.add(144,-13.69004962);
        angleCalculator.add(160,-16.42930145);
        angleCalculator.add(174,-19.093492);
//        angleCalculator.add(192,0);
//        angleCalculator.add(208,0);
//        angleCalculator.add(216,0);
//        angleCalculator.add(232,0);
//        angleCalculator.add(248,0);
//        angleCalculator.add(264,0);
//        angleCalculator.add(280,0);
        angleCalculator.createLUT();
    }
    /** variables that will be reused for calculations **/
    private Mat mat;
    private Mat ret;

    private final InterpLUT areaPerpendicular;
    private final InterpLUT angleCalculator;
    private LinearOpMode linearOpMode;
    private ArrayList<Pose2d> vectors = new ArrayList<>();

    Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);
    private double x;
    private double y;
    double width;
    double height;

    /** width of the camera in use, defaulted to 320 as that is most common in examples **/
    public static int CAMERA_WIDTH = 320;

    /** Horizon value in use, anything above this value (less than the value) since
     * (0, 0) is the top left of the camera frame **/
    public static int HORIZON = (int)((100.0 / 320.0) * CAMERA_WIDTH);
    /** if the calculated aspect ratio is greater then this, height is 4, otherwise its 1 **/
    final double BOUND_RATIO = 0.7;

    @Override
    public Mat processFrame(Mat input) {
        ret.release(); // releasing mat to release backing buffer
        // must release at the start of function since this is the variable being returned

        ret = new Mat(); // resetting pointer held in ret
        try { // try catch in order for opMode to not crash and force a restart
            /**converting from RGB color space to YCrCb color space**/
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            /**checking if any pixel is within the orange bounds to make a black and white mask**/
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in
            Core.inRange(mat, lowerOrange, upperOrange, mask);

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask);

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            /**finding contours on mask**/
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);
            vectors = new ArrayList<>();
            for (MatOfPoint c: contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect =  Imgproc.boundingRect(copy);
                // checking if the rectangle is below the horizon
                if (rect.y + rect.height > HORIZON) {
                    width = rect.width;
                    height = rect.height;
                    x = rect.x;
                    y = rect.y;
                    double angle = Math.toRadians(angleCalculator.get(rect.x));
                    double distance = (areaPerpendicular.get(rect.area())+ 8.5)/Math.cos(angle);
                    vectors.add(new Pose2d(0, distance, angle));
                    //Imgproc.rectangle(ret, maxRect, new Scalar(0.0, 0.0, 255.0), 2);
                }
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }
            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
            Imgproc.line(
                    ret,
                    new Point(
                            .0,
                            HORIZON
                    ),
                    new Point(
                            CAMERA_WIDTH,
                            HORIZON
                    ),
                    new Scalar(
                            255.0,
                            .0,
                            255.0)
            );

//            if (debug) telemetry?.addData("Vision: maxW", maxWidth)

            /** checking if widest width is greater than equal to minimum width
             * using Kotlin if expression (Java ternary) to set height variable
             *
             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
             **/
//            height = if (maxWidth >= MIN_WIDTH) {
//                val aspectRatio: Double = maxRect.height.toDouble() / maxRect.width.toDouble()
//
//                if(debug) telemetry?.addData("Vision: Aspect Ratio", aspectRatio)
//
//                /** checks if aspectRatio is greater than BOUND_RATIO
//                 * to determine whether stack is ONE or FOUR
//                 */
//                if (aspectRatio > BOUND_RATIO)
//                    Height.FOUR // height variable is now FOUR
//                else
//                    Height.ONE // height variable is now ONE
//            } else {
//                Height.ZERO // height variable is now ZERO
//            }
//
//            if (debug) telemetry?.addData("Vision: Height", height)

            // releasing all mats after use
            mat.release();
            mask.release();
            hierarchy.release();

        } catch (Exception e) {
            e.printStackTrace();
        }
        return ret;
    }
    public ArrayList<Vector2d> getVectors(Pose2d currentPose){
        ArrayList<Vector2d> list = new ArrayList<>();
        Coordinate current = new Coordinate(currentPose);
        for(Pose2d pose2d : vectors){
            Coordinate temp = current.toPoint();
            temp.polarAdd(currentPose.getHeading() - Math.PI, 3.5);
            temp.polarAdd(pose2d.getHeading() + currentPose.getHeading(), pose2d.getY());
            list.add(temp.toPose2d(0).vec());
        }
        return list;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getArea(){
        return width*height;
    }
}
