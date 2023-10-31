package org.firstinspires.ftc.teamcode;

import android.graphics.ColorFilter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.tensorflow.lite.task.vision.segmenter.ColoredLabel;

import java.util.List;

@Autonomous(name = "FrontStageBlue")

public class FrontStageBlue extends LinearOpMode {

    private VisionPortal visionPortal;
    private Spike_Detector det;

    @Override
    public void runOpMode() {
        waitForStart();
        initVision();
        visionPortal.resumeStreaming();

        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        try {
            if (opModeIsActive()) {
                int spike = det.getCorrectSpike();
                while (opModeIsActive() && spike == 0) {
                    spike = det.getCorrectSpike();
                }

                if (!opModeIsActive()) {
                    return;
                }

                // Stop using the camera
                visionPortal.stopStreaming();

//                if (spike == 1) {
//                    driver.drive(36);
//                    driver.turn(-90);
//                    driver.drive(-8);
//                    //spit
//                    driver.drive(8);
//                    driver.turn(90);
//                    driver.drive(14);
//                    //what else
//                } else if (spike == 2) {
//                    driver.drive(40);
//                    //spit
//                    driver.drive(10);
//                    //what else
//                } else if (spike == 3) {
//                    driver.drive(36);
//                    driver.turn(90);
//                    driver.drive(-8);
//                    //spit
//                    driver.drive(8);
//                    driver.turn(-90);
//                    driver.drive(14);
//                    //what else
//                }

            }
        } finally {
            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();
            driver.stuff(0, 0, 0);
        }

    //private void telemetryAprilTag() {

  //      List<ColoredLabel> currentDetections = det.getCorrectSpike();
     //   telemetry.addData("# Correct Spikes Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
       // for (AprilTagDetection detection : currentDetections) {
         //   if (detection.metadata != null) {
           //     telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
             //   telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
               // telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
               // telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
         //   } else {
         //       telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
         //   }

         //   if (detection.id == 1 && detection.ftcPose != null) {
         //       final TelemetryPacket packet = new TelemetryPacket();
           //     packet.put("x-pos", detection.ftcPose.x);

             //   FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }   // end for() loop


        // Add "key" information to telemetry
    //    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
    //    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    //    telemetry.addLine("RBE = Range, Bearing & Elevation");

  //  }

    private void initVision() {
        det = new Spike_Detector();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, ConceptAprilTagEasy.CamOnly.webcam), det);

    }
}