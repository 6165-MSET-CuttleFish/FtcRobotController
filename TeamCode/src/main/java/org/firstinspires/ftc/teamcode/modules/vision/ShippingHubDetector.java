package org.firstinspires.ftc.teamcode.modules.vision;


import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
@Config
public class ShippingHubDetector extends OpenCvPipeline {
    public ShippingHubDetector(){
        ret = new Mat();
        mat = new Mat();

    }

    private Mat mat;
    private Mat ret;
    Scalar lowerOrange = new Scalar(0.0, 128.0, 128.0); //
    Scalar upperOrange = new Scalar(128.0, 128.0, 128.0);
    private double x;
    private double y;
    double width;
    double height;
    public static int CAMERA_WIDTH = 320;
    public static int HORIZON = (int)((100.0 / 320.0) * CAMERA_WIDTH);
    public static boolean showOriginal;
    @Override
    public Mat processFrame(Mat input) {
        ret.release(); // releasing mat to release backing buffer
        // must release at the start of function since this is the variable being returned

        ret = new Mat(); // resetting pointer held in ret
        try { // try catch in order for opMode to not crash and force a restart
            /**converting from RGB color space to YCrCb color space**/
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2Lab); //black


            /**checking if any pixel is within the orange bounds to make a black and white mask**/
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_16F); // variable to store mask in
            Core.inRange(mat, lowerOrange, upperOrange, mask);

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask);

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, new Size(25.0, 25.0), 0.00);

            /**finding contours on mask**/
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);
            for (MatOfPoint c: contours) {
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                RotatedRect ellipse =  Imgproc.fitEllipse(copy);
                // checking if the rectangle is below the horizon
                if (ellipse.center.y + ellipse.size.height > HORIZON) {
                    width = ellipse.size.width;
                    height = ellipse.size.height;
                    x = ellipse.center.x;
                    y = ellipse.center.y;
                    Imgproc.ellipse(ret, ellipse, new Scalar(0.0, 0.0, 255.0), 2);
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
        return showOriginal ? input : ret;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
}


