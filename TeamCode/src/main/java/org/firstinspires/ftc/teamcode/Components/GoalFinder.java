package org.firstinspires.ftc.teamcode.Components;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class GoalFinder extends OpenCvPipeline {
    public GoalFinder(){
        ret = new Mat();
        mat = new Mat();
    }
    /** variables that will be reused for calculations **/
    private Mat mat;
    private Mat ret;

    Scalar lowerBlue = new Scalar(0.0, 141.0, 0.0);
    Scalar upperBlue = new Scalar(140, 220.0, 255.0);

    Scalar lowerRed = new Scalar(0.0, 141.0, 0.0);
    Scalar upperRed = new Scalar(140, 220.0, 255.0);
    private double x;
    double width;
    double height;
    public static int CAMERA_WIDTH = 320;
    public static int HORIZON = (int)((100.0 / 320.0) * CAMERA_WIDTH);

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
            switch (Details.side){
                case BLUE: Core.inRange(mat, lowerBlue, upperBlue, mask);
                case RED: Core.inRange(mat, lowerRed, upperRed, mask);
            }

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
            for (MatOfPoint c: contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect =  Imgproc.boundingRect(copy);
                // checking if the rectangle is below the horizon
                if (rect.y + rect.height > HORIZON) {
                    width = rect.width;
                    height = rect.height;
                    x = rect.x;
                    Imgproc.rectangle(ret, rect, new Scalar(0.0, 0.0, 255.0), 2);
                }
                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }
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
            mat.release();
            mask.release();
            hierarchy.release();

        } catch (Exception e) {
            e.printStackTrace();
        }
        return ret;
    }
    public double getX(){
        return x;
    }
}
