package org.firstinspires.ftc.teamcode.modules.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
//
public class detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    private int location = -1;
    // find and set the three regions of interest
    static final Rect POS_1 = new Rect(
            new Point(0, 0),
            new Point(0, 0));
    static final Rect POS_2 = new Rect(
            new Point(0, 0),
            new Point(0, 0));
    static final Rect POS_3 = new Rect(
            new Point(0, 0),
            new Point(0, 0));
    static double PERCENT_COLOR_THRESH = 0;

    public detector(Telemetry t){ telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // find and set the low and high color thresholds
        Scalar lowHSV = new Scalar(0, 0, 0);
        Scalar highHSV = new Scalar(0, 0, 0);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat pos_1 = mat.submat(POS_1);
        Mat pos_2 = mat.submat(POS_2);
        Mat pos_3 = mat.submat(POS_3);

        double pos_1_val = Core.sumElems(pos_1).val[0] / POS_1.area() / 255;
        double pos_2_val = Core.sumElems(pos_2).val[0] / POS_2.area() / 255;
        double pos_3_val = Core.sumElems(pos_3).val[0] / POS_3.area() / 255;

        pos_1.release();
        pos_2.release();
        pos_3.release();

        telemetry.addData("Position 1 raw", (int) Core.sumElems(pos_1).val[0]);
        telemetry.addData("Position 2 raw", (int) Core.sumElems(pos_2).val[0]);
        telemetry.addData("Position 3 raw", (int) Core.sumElems(pos_3).val[0]);
        telemetry.addData("Position 1 percent", Math.round(pos_1_val*100));
        telemetry.addData("Position 2 percent", Math.round(pos_2_val*100));
        telemetry.addData("Position 3 percent", Math.round(pos_3_val*100));

        double maxVal = Math.max(Math.max(pos_1_val, pos_2_val), pos_3_val);

        if(maxVal == pos_1_val) location = 1;
        else if(maxVal == pos_2_val) location = 2;
        else location = 3;

        telemetry.addData("Duck Location", location);
        telemetry.update();

        return mat;
    }
}
