package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.util.field.Context.alliance;
import static org.firstinspires.ftc.teamcode.util.field.Context.telemetry;
import static org.opencv.imgproc.Imgproc.rectangle;

//
@Config
public class TSEDetector extends OpenCvPipeline {
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
    }
    private Location location = Location.MIDDLE;
    // find and set the three regions of interest
    public static  Rect POS_1 = new Rect(30, 150, 80, 50);
    public static  Rect POS_2 = new Rect(160, 160, 80, 50);

    public static Rect POS_1_BLUE = new Rect(80, 150, 80, 50);
    public static  Rect POS_2_BLUE = new Rect(200, 160, 80, 50);
//    public static  Rect POS_3 = new Rect(
//            new Point(230, 130),
//            new Point(280, 210));
//    public static  Rect test = new Rect(
//            new Point(10, 10),
//            new Point(50, 50));

    public static int redHLow = 0;
    public static int redSLow = 50;
    public static int redVLow = 160;

    public static int redHHigh = 180;
    public static int redSHigh = 255;
    public static int redVHigh = 255;

    public static int blueHLow = 100;
    public static int blueSLow = 50;
    public static int blueVLow = 110;

    public static int blueHHigh = 130;
    public static int blueSHigh = 255;
    public static int blueVHigh = 255;

    public static int visionMax = 100;
    public static int visionMin = 12;

    public static boolean returnBlack = true;

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSV;
        Scalar highHSV;
        if (alliance == Alliance.BLUE) {
            lowHSV = new Scalar(blueHLow, blueSLow, blueVLow);
            highHSV = new Scalar(blueHHigh, blueSHigh, blueVHigh);
        } else {
            lowHSV = new Scalar(redHLow, redSLow, redVLow);
            highHSV = new Scalar(redHHigh, redSHigh, redVHigh);
        }
        Core.inRange(mat, lowHSV, highHSV, mat);


        Mat pos_1 = mat.submat(alliance == Alliance.BLUE ? POS_1_BLUE : POS_1);
        Mat pos_2 = mat.submat(alliance == Alliance.BLUE ? POS_2_BLUE : POS_2);
        //Mat pos_3 = mat.submat(POS_3);

        rectangle(mat, alliance == Alliance.BLUE ? POS_1_BLUE : POS_1, new Scalar(255, 255, 255));
        rectangle(mat, alliance == Alliance.BLUE ? POS_2_BLUE : POS_2, new Scalar(255, 255, 255));
        //rectangle(mat, POS_3, new Scalar(255, 255, 255));

        rectangle(input, alliance == Alliance.BLUE ? POS_1_BLUE : POS_1, new Scalar(255, 255, 255));
        rectangle(input, alliance == Alliance.BLUE ? POS_2_BLUE : POS_2, new Scalar(255, 255, 255));
        //rectangle(input, POS_3, new Scalar(255, 255, 255));

        //percent of pixels that are in range within area
        double pos_1_val = Core.sumElems(pos_1).val[0] / (alliance == Alliance.BLUE ? POS_1_BLUE.area() : POS_1.area()) / 255;
        double pos_2_val = Core.sumElems(pos_2).val[0] / (alliance == Alliance.BLUE ? POS_2_BLUE.area() : POS_2.area()) / 255;
        //double pos_3_val = Core.sumElems(pos_3).val[0] / POS_3.area() / 255;

        double pos_1_percent = Math.round(pos_1_val*100);
        double pos_2_percent = Math.round(pos_2_val*100);

        if (telemetry != null)
//        telemetry.addData("Position 1 raw", (int) Core.sumElems(pos_1).val[0]);
//        telemetry.addData("Position 2 raw", (int) Core.sumElems(pos_2).val[0]);
        //telemetry.addData("Position 3 raw", (int) Core.sumElems(pos_3).val[0]);
//        telemetry.addData("Position 1 percent", Math.round(pos_1_val*100));
//        telemetry.addData("Position 2 percent", Math.round(pos_2_val*100));
        //telemetry.addData("Position 3 percent", Math.round(pos_3_val*100));

        pos_1.release();
        pos_2.release();
        //pos_3.release();
        if (alliance == Alliance.RED) {
            boolean leftFull = pos_1_percent > visionMin && pos_1_percent < visionMax;
            boolean midFull = pos_2_percent > visionMin && pos_2_percent < visionMax;
            if (leftFull && midFull) location = Location.RIGHT;
            else if (leftFull) location = Location.MIDDLE;
            else location = Location.LEFT;
        } else {
            boolean rightFull = pos_2_percent > visionMin && pos_2_percent < visionMax;
            boolean midFull = pos_1_percent > visionMin && pos_1_percent < visionMax;
            if (rightFull && midFull) location = Location.LEFT;
            else if (rightFull) location = Location.MIDDLE;
            else location = Location.RIGHT;
        }


//        double maxVal = Math.max(Math.max(pos_1_val, pos_2_val), pos_3_val);
//
//        if (maxVal == pos_1_val) location = Location.RIGHT;
//        else if (maxVal == pos_2_val) location = Location.MIDDLE;
//        else location = Location.LEFT;

        // telemetry.addData("location", getLocation());

        return returnBlack ? mat : input;
    }

    public Location getLocation() {
        return location;
    }
}
