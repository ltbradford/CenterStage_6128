package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test Wheels", group="TEST")
public class Test_Wheels extends LinearOpMode {

    //This OpMode is simply meant to test each wheel individually to see if things got wired
    //correctly and for correct mapping.
    @Override
    public void runOpMode() {
        Autodrive driver = new Autodrive(hardwareMap);

        // Initialize the hardware variables. Note that the strings (names assigned) here must
        // correspond to the names assigned in the robot configuration tab on the driver hub.
        // In this case, we're using the hardwareMap from Autodrive, so we can adjust everything
        // in that class, as needed, during testing.

        // Wait for the game to start (driver presses initiate, then play)
        waitForStart();

        while (opModeIsActive()) {

            // This will run the left front wheel power (slowly) while the square
            // button is pressed on the game controller. Do the same for each of
            // the other wheels; assigning each to their own button to test individually.
            if (gamepad1.square) {
                driver.leftFrontDrive.setPower(0.2);
            } else driver.leftFrontDrive.setPower(0);

            if (gamepad1.triangle) {
                driver.leftBackDrive.setPower(0.2);
            } else driver.leftBackDrive.setPower(0);

            if (gamepad1.circle) {
                driver.rightFrontDrive.setPower(0.2);
            } else driver.rightFrontDrive.setPower(0);

            if (gamepad1.cross) {
                driver.rightBackDrive.setPower(0.2);
            } else driver.rightBackDrive.setPower(0);
        }
        //Tip from Chase: Can also turn this ternary operation to do the same as above.
        // <condition> ? <value if true> : <value if false>
        // driver.leftBackDrive.setPower(gamepad1.triangle ? 0.2 : 0);

    }
}
