package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class Spike_Detector implements VisionProcessor {
        private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
        private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
        private PipelineRecordingParameters pipeline;
        private OpenCvCamera camera;

    int correctSpike = 0;

    public int getCorrectSpike() {

        return correctSpike;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Do initialization if needed. Swap cameras, multiple cameras....?
    }

    @Override
    public Object processFrame(Mat bgrimage, long captureTimeNanos) {
        // Given a frame from camera.

        Point bottomLeft1 = new Point(0, 0);
        Point topRight1 = new Point(80, 480);
        Point bottomLeft2 = new Point(100, 220);
        Point topRight2 = new Point(580, 480);
        Point bottomLeft3 = new Point(400, 0);
        Point topRight3 = new Point(640, 480);

        Scalar lower = new Scalar(0, 0, 0);
        Scalar upper = new Scalar(255, 255, 255);


        //Mat cropped = bgrimage.colRange(left, right).rowRange(top, bottom);
        Mat hsvimage = new Mat();

        // Convert frame from BGR to HSV to improve color finding.
       // Imgproc.cvtColor(cropped, hsvimage, Imgproc.COLOR_BGR2HSV);

        Mat binaryMat = new Mat();

        Core.inRange(hsvimage, lower, upper, binaryMat);
        // Finds colors in range and places results in a separate image.

        double w1 = 0;
        double w2 = 0;
        double w3 = 0;

        // process the pixel value for each rectangle  (255 = white, 0 = black)
        for (int i = (int) topRight1.x; i >= bottomLeft1.x; --i) {
            for (int j = (int) topRight1.y; j >= bottomLeft1.y; --j) {
                if (binaryMat.get(i, j)[0] == 255) {
                    ++w1;
                }
            }
        }

        for (int i = (int) topRight2.x; i >= bottomLeft2.x; --i) {
            for (int j = (int) topRight2.y; j >= bottomLeft2.y; --j) {
                if (binaryMat.get(i, j)[0] == 255) {
                    ++w2;
                }
            }
        }

        for (int i = (int) topRight3.x; i >= bottomLeft3.x; --i) {
            for (int j = (int) topRight3.y; j >= bottomLeft3.y; --j) {
                if (binaryMat.get(i, j)[0] == 255) {
                    ++w3;
                }
            }
        }

        if (w1 > w2 && w1 > w3) {
            correctSpike = 1;
        } else if (w2 > w1 && w2 > w3) {
            correctSpike = 2;
        } else if (w3 > w1 && w3 > w2){
            correctSpike = 3;
        }

        return null;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}