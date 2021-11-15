package org.firstinspires.ftc.teamcode.modules.vision;

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
public class Detector extends OpenCvPipeline {
    Telemetry telemetry;

    private int location = -1;
    // find and set the three regions of interest
    static final Rect POS_1 = new Rect(
            new Point(20, 10),
            new Point(100, 100));
    static final Rect POS_2 = new Rect(
            new Point(120, 10),
            new Point(200, 100));
    static final Rect POS_3 = new Rect(
            new Point(220, 10),
            new Point(300, 100));
    static final Rect test = new Rect(
            new Point(10, 10),
            new Point(50, 50));
    static double PERCENT_COLOR_THRESH = 0;

    public Detector(Telemetry t){ telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // find and set the low and high color thresholds
//        Scalar lowHSV = new Scalar(70, 190, 110);
//        Scalar highHSV = new Scalar(110, 230, 150);

        Scalar lowHSV = new Scalar(50, 50, 110);
        Scalar highHSV = new Scalar(90, 90, 150);

        rectangle(mat, POS_1, new Scalar(255, 0, 0));
        rectangle(mat, POS_2, new Scalar(255, 0, 0));
        rectangle(mat, POS_3, new Scalar(255, 0, 0));

        Mat pos_1 = mat.submat(POS_1);
        Mat pos_2 = mat.submat(POS_2);
        Mat pos_3 = mat.submat(POS_3);
//        rectangle(mat, test, new Scalar(0, 0, 255));
//        Mat test_ = mat.submat(test);
//        telemetry.addData("test avg h", Core.sumElems(test_).val[0]/test.area());
//        telemetry.addData("test avg s", Core.sumElems(test_).val[1]/test.area());
//        telemetry.addData("test avg v", Core.sumElems(test_).val[2]/test.area());
//        //67, 67, 130


        Core.inRange(mat, lowHSV, highHSV, mat);

        //percent of pixels that are in range within area
        double pos_1_val = Core.sumElems(pos_1).val[2] / POS_1.area() / 255;
        double pos_2_val = Core.sumElems(pos_2).val[2] / POS_2.area() / 255;
        double pos_3_val = Core.sumElems(pos_3).val[2] / POS_3.area() / 255;

        telemetry.addData("Position 1 raw", (int) Core.sumElems(pos_1).val[2]);
        telemetry.addData("Position 2 raw", (int) Core.sumElems(pos_2).val[2]);
        telemetry.addData("Position 3 raw", (int) Core.sumElems(pos_3).val[2]);
        telemetry.addData("Position 1 percent", Math.round(pos_1_val*100));
        telemetry.addData("Position 2 percent", Math.round(pos_2_val*100));
        telemetry.addData("Position 3 percent", Math.round(pos_3_val*100));

        pos_1.release();
        pos_2.release();
        pos_3.release();

        double minVal = Math.min(Math.min(pos_1_val, pos_2_val), pos_3_val);

        if(minVal == pos_1_val) location = 1;
        else if(minVal == pos_2_val) location = 2;
        else location = 3;

        telemetry.addData("Duck Location", location);
        telemetry.update();

        return mat;
    }

}
