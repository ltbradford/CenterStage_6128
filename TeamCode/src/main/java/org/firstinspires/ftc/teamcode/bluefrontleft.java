package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "blue front left")
public class bluefrontleft extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap);

        boolean isBlue = true;

        Spike_Detector_2 detector = new Spike_Detector_2(isBlue);

        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam"),
                detector);

        visionPortal.resumeStreaming();
        int correctSpike = 0;

        while(correctSpike == 0) {
            correctSpike = detector.getCorrectSpike();
        }
        visionPortal.stopStreaming();

        sleep(5000);

        // For left spike auto instructions.
        if (correctSpike == 1) {
            driver.drive(34);
            driver.turn(90);
            driver.drive(-5);
            //feeder spits out pixel
            sleep(2000);
            driver.drive(5);
            driver.turn(-90);
            driver.drive(22);
        }
           // For middle spike auto instructions.
            if (correctSpike == 2) {
                driver.drive(50);
                //feeder spits out pixel
                sleep(2000);
                driver.drive(6);
        }
            // For right spike auto instructions.
            if (correctSpike == 3) {
                driver.drive(34);
                driver.turn(-90);
                driver.drive(-5);
                //feeder spits out pixel
                sleep(2000);
                driver.drive(5);
                driver.turn(90);
                driver.drive(22);
            }

            // Now we continue on with the autonomous driving instructions to get the robot
            // through the middle racks, then over to the correct backdrop to read April Tags.
            driver.turn(-90);
            driver.drive(80);

            if (correctSpike == 1) {
                driver.turn(90);
                driver.drive(18);
            }

// You can use the code for detecting the April Tags, instead of simply using the Spike Detector
// to navigate to the correct spot on the backdrop manually.
     //   AprilTag_Detector targetTag = new AprilTag_Detector();
        // For matching the spike # to the april tag #.
     //   if (correctSpike == 1

        }

    }

