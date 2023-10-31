package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class Spike_Detector implements VisionProcessor {
        private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
        private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

        private static final int HORIZON = 100; // horizon value to tune

        private static final boolean DEBUG = false; // if debug is wanted, change to true

        private static final boolean USING_WEBCAM = true; // change to true if using webcam
        private static final String WEBCAM_NAME = "webcam"; // insert webcam name from configuration if using webcam

        private PipelineRecordingParameters pipeline;
        private OpenCvCamera camera;

    public static final int top = 0;
    public static final int left = 0;
    public static final int bottom = 0;
    public static final int right = 0;

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

        Mat cropped = bgrimage.colRange(left, right).rowRange(top, bottom);
        Mat hsvimage = new Mat();

        // Convert frame from BGR to HSV to improve color finding.
        Imgproc.cvtColor(cropped, hsvimage, Imgproc.COLOR_BGR2HSV);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

//        canvas.drawRect();
    }
}