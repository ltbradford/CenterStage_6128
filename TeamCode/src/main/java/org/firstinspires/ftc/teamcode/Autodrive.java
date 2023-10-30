package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * basic autonomous movement "blueprints" for driving and turning.
 * Used mainly to be able to get used by other files instead of writing
 * the drive and turn codes again.
 */
@Config
public class Autodrive {

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    private final IMU imu;

    public static int TICKS_PER_INCH = 40;

    public static double MIN_POWER_TO_MOVE = 0.0;

    public static double turnGain =-0.02;

    public static double driveGain =0.01;

    public static double minturnpower = 0.45;

    public Autodrive(DcMotor leftFrontDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive, DcMotor rightBackDrive, IMU imu) {
        this.leftFrontDrive = leftFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.imu = imu;
    }

    public Autodrive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void drive(int distanceInches) {
        int ticksDistance = 4 * (distanceInches * TICKS_PER_INCH);

        final int startingPosition =
                        leftBackDrive.getCurrentPosition() +
                        rightBackDrive.getCurrentPosition() +
                        leftFrontDrive.getCurrentPosition() +
                        rightFrontDrive.getCurrentPosition();

        int targetPosition = startingPosition + ticksDistance;

        //stop if within 0.5 inch
        int tolerance = TICKS_PER_INCH/2;

        int error = targetPosition - startingPosition;

        while ( Math.abs(error) > tolerance ){

            double axial = error * driveGain;

            // If the magnitude of axial power is less than the min power to move,
            // then adjust will be greater than 1.0. Scale axial power without changing
            // it's sign to ensure it's strong enough.
            // If scale is less than 1, then don't make the power any weaker.

            double adjust = MIN_POWER_TO_MOVE/ Math.abs(axial);

            if (adjust > 1.0) {
                axial *= adjust;
            }
            stuff (axial, 0, 0);

            int currentPos =
                    leftBackDrive.getCurrentPosition() +
                    rightBackDrive.getCurrentPosition() +
                    leftFrontDrive.getCurrentPosition() +
                    rightFrontDrive.getCurrentPosition();

            error = targetPosition - currentPos;

            TelemetryPacket stats = new TelemetryPacket();
            stats.put("Axial", error);
            FtcDashboard.getInstance().sendTelemetryPacket(stats);
        }

        // Stop the motors. We made it.
        stuff(0, 0, 0);
    }

    public void turn(final double degrees) {
        imu.resetYaw();

        //stop if within 3 degrees
        int tolerance = 3;

        double error = degrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while ( Math.abs(error) > tolerance ) {
            double yaw = error * turnGain;

            // If the magnitude of yaw power is less than the min turn power,
            // then adjust will be greater than 1.0. Scale yaw without changing
            // it's sign to ensure it's strong enough.
            // If scale is less than 1, then don't make the power any weaker.
            double adjust = minturnpower / Math.abs(yaw);
            if (adjust > 1.0) {
                yaw *= adjust;
            }

            stuff(0, 0, yaw);
            error = degrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            TelemetryPacket stats = new TelemetryPacket();
            stats.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            FtcDashboard.getInstance().sendTelemetryPacket(stats);

        }

        // Stop the motors. We made it.
        stuff(0, 0, 0);
    }

    public void stuff(double axial, double lateral, double yaw) {
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower   = leftBackPower / max;
            rightBackPower  = rightBackPower / max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


}