package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "blue front left")
public class bluefrontleft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront");
        DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        Autodrive driver = new Autodrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, imu);
        driver.drive(36);
        driver.turn(85);

        while (opModeIsActive()) {
            TelemetryPacket stats = new TelemetryPacket();
            stats.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            FtcDashboard.getInstance().sendTelemetryPacket(stats);
        }
    }
}
