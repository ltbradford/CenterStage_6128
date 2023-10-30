package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "blue front left")
public class bluefrontleft extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap);
        driver.drive(35);
        driver.turn(90);
    }
}
