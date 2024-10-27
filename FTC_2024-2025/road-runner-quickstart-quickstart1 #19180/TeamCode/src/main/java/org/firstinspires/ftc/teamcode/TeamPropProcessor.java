package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.net.wifi.aware.IdentityChangedListener;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import kotlin.reflect.KDeclarationContainer;

public class TeamPropProcessor implements VisionProcessor {

    Rect LEFT_RECTANGLE;
    Rect MIDDLE_RECTANGLE;
    Rect RIGHT_RECTANGLE;


    Mat hsvMat = new Mat();
    Mat lowMat = new Mat();
    Mat highMat = new Mat();
    Mat detectedMat = new Mat();

    Scalar lowerRedThresholdLow = new Scalar(0, 125, 125);
    Scalar LowerRedThresholdHigh = new Scalar(10, 255, 255);
    Scalar upperRedRThresholdLow = new Scalar(165, 125, 125);
    Scalar upperRedRThresholdHigh = new Scalar(180, 255, 255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        LEFT_RECTANGLE = new Rect(
                new Point(0, 0.25 * height),
                new Point(0.33 * width, height)
        );
        MIDDLE_RECTANGLE = new Rect(
                new Point(0.33 * width, 0),
                new Point(0.66 * width, height)
        );
        RIGHT_RECTANGLE = new Rect(
                new Point(0.66 * width, 0),
                new Point(width, height)
        );
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        // RED HSV
        // S: ~50%,V: ~50%
        // H:  0-20, 34-360

        Core.inRange(hsvMat, lowerRedThresholdLow, LowerRedThresholdHigh, lowMat);
        Core.inRange(hsvMat, upperRedRThresholdLow, upperRedRThresholdHigh, highMat);

        Core.bitwise_or(lowMat, highMat, detectedMat);

        double leftPercent = Core.sumElems(detectedMat.submat(LEFT_RECTANGLE)).val[0] / 255;
        double middlePercent = Core.sumElems(detectedMat.submat(MIDDLE_RECTANGLE)).val[0] / 255;
        double rightPercent = Core.sumElems(detectedMat.submat(RIGHT_RECTANGLE)).val[0] / 255;

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
