package org.firstinspires.ftc.teamcode.modules.vision;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.imgproc.Imgproc.rectangle;

//
@Config
public class Detector extends OpenCvPipeline {
    Telemetry telemetry;

    private int location = -1;
    // find and set the three regions of interest
    public static  Rect POS_1 = new Rect(
            new Point(90, 130),
            new Point(140, 210));
    public static  Rect POS_2 = new Rect(
            new Point(160, 130),
            new Point(210, 210));
    public static  Rect POS_3 = new Rect(
            new Point(230, 130),
            new Point(280, 210));
    public static  Rect test = new Rect(
            new Point(10, 10),
            new Point(50, 50));
    static double PERCENT_COLOR_THRESH = 0;

    public static int hLow = 50;
    public static int sLow = 50;
    public static int vLow = 110;

    public static int hHigh = 90;
    public static int sHigh = 110;
    public static int vHigh = 255;

    public Detector(Telemetry t){ telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // find and set the low and high color thresholds
//        Scalar lowHSV = new Scalar(70, 190, 110);
//        Scalar highHSV = new Scalar(110, 230, 150);

        Scalar lowHSV = new Scalar(hLow, sLow, vLow);
        Scalar highHSV = new Scalar(hHigh, sHigh, vHigh);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat pos_1 = mat.submat(POS_1);
        Mat pos_2 = mat.submat(POS_2);
        Mat pos_3 = mat.submat(POS_3);
        //rectangle(mat, test, new Scalar(0, 0, 255));
//        Mat test_ = mat.submat(test);
//        telemetry.addData("test avg h", Core.sumElems(test_).val[0]/test.area());
//        telemetry.addData("test avg s", Core.sumElems(test_).val[1]/test.area());
//        telemetry.addData("test avg v", Core.sumElems(test_).val[2]/test.area());
//        //67, 67, 130

        rectangle(mat, POS_1, new Scalar(255, 255, 255));
        rectangle(mat, POS_2, new Scalar(255, 255, 255));
        rectangle(mat, POS_3, new Scalar(255, 255, 255));

        //percent of pixels that are in range within area
        double pos_1_val = Core.sumElems(pos_1).val[0] / POS_1.area() / 255;
        double pos_2_val = Core.sumElems(pos_2).val[0] / POS_2.area() / 255;
        double pos_3_val = Core.sumElems(pos_3).val[0] / POS_3.area() / 255;

//        double hAvg = Core.sumElems(pos_2).val[0] / POS_1.area() / 255;
//        double sAvg = Core.sumElems(pos_2).val[1] / POS_2.area() / 255;
//        double vAvg = Core.sumElems(pos_2).val[2] / POS_3.area() / 255;

        telemetry.addData("Position 1 raw", (int) Core.sumElems(pos_1).val[0]);
        telemetry.addData("Position 2 raw", (int) Core.sumElems(pos_2).val[0]);
        telemetry.addData("Position 3 raw", (int) Core.sumElems(pos_3).val[0]);
        telemetry.addData("Position 1 percent", Math.round(pos_1_val*100));
        telemetry.addData("Position 2 percent", Math.round(pos_2_val*100));
        telemetry.addData("Position 3 percent", Math.round(pos_3_val*100));
//
//        telemetry.addData("h avg", Math.round(hAvg*100));
//        telemetry.addData("s avg", Math.round(sAvg*100));
//        telemetry.addData("v avg", Math.round(vAvg*100));

        pos_1.release();
        pos_2.release();
        pos_3.release();

        double maxVal = Math.max(Math.max(pos_1_val, pos_2_val), pos_3_val);

        if(maxVal == pos_1_val) location = 1;
        else if(maxVal == pos_2_val) location = 2;
        else location = 3;

        telemetry.addData("Duck Location", location);
        telemetry.update();

        return mat;
    }

}
