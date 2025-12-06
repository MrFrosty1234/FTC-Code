package org.woen.Modules.Camera;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.resize;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;


@Config
public class DetectColor implements VisionProcessor {


    double x = 640;

    double y = 480;


    Mat imgGreen = new Mat();
    Mat imgPurple = new Mat();

    public static double hGreenUp = 0;
    public static double cGreenUp = 0;
    public static double vGreenUp = 0;
    public static double hGreenDown = 0;
    public static double cGreenDown = 0;
    public static double vGreenDown = 0;

    public static double hPurpleUp = 0;
    public static double cPurpleUp = 0;
    public static double vPurpleUp = 0;
    public static double hPurpleDown = 0;
    public static double cPurpleDown = 0;
    public static double vPurpleDown = 0;

    double x1Start = x * 0;
    double x1Finish = x * 0.4;
    double x2Start = x * 0.4;
    double x2Finish = x * 0.7;
    double x3Start = x * 0.7;
    double x3Finish = x * 1;



    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        cvtColor(frame, frame, COLOR_RGB2HSV);
        resize(frame, frame, new Size(x,y));
        new Mat(frame, new Rect(0,0, (int) x/2, (int) y/2)).copyTo(frame);
        inRange(frame,
                new Scalar(hGreenDown, cGreenDown, vGreenDown),
                new Scalar(hGreenUp, cGreenUp, vGreenUp),
                imgGreen);
        inRange(frame,
                new Scalar(hPurpleDown, cPurpleDown, vPurpleDown),
                new Scalar(hPurpleUp, cPurpleUp, vPurpleUp),
                imgPurple);


        Core.bitwise_or(imgGreen, imgPurple, frame);

        Moments momentsPurple = Imgproc.moments(imgPurple);

        double cxPurple = momentsPurple.m10/momentsPurple.m00;

        if (cxPurple <= 240) {
            return 1;
        }
        if (cxPurple < 443 && cxPurple >= 240) {
            return 2;
        }
        if (cxPurple >= 443) {
            return 3;
        }

        Moments momentsGreen= Imgproc.moments(imgGreen);

        double cxGreen = momentsGreen.m10/momentsPurple.m00;

        if (cxGreen <= 240) {
            return 4;
        }
        if (cxGreen < 443 && cxPurple >= 240) {
            return 5;
        }
        if (cxGreen >= 443) {
            return 6;
        }


        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
