package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveHardware;

@TeleOp(name = "TELEOP_OPEN_LOOP_TEST", group = "Test")
@Config
@Disabled
public class TELEOP_AutoShort_TESTING extends LinearOpMode {


    public static double DRIVE_POWER = 0.6;
    public static double STRAFE_POWER = 0.6;
    public static double TURN_POWER = 0.5;

    public static double CM_PER_SECOND = 55.0;
    public static double DEG_PER_SECOND = 120.0;
    public static double TEST_DISTANCE_CM = 30;
    public static double TEST_TURN_DEG = 30;

    DriveHardware drive;

    @Override
    public void runOpMode() {

        drive = new DriveHardware(hardwareMap);

        telemetry.addLine("OPEN LOOP MODE (NO IMU / NO ODOM)");
        telemetry.addLine("Dashboard tunable");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) moveStraight(TEST_DISTANCE_CM);
            if (gamepad1.dpad_down) moveStraight(-TEST_DISTANCE_CM);

            if (gamepad1.dpad_left) strafe(TEST_DISTANCE_CM);
            if (gamepad1.dpad_right) strafe(-TEST_DISTANCE_CM);

            if (gamepad1.left_bumper) turnRelative(TEST_TURN_DEG);
            if (gamepad1.right_bumper) turnRelative(-TEST_TURN_DEG);

            idle();
        }
    }
    void moveStraight(double cm) {
        double timeMs = Math.abs(cm) / CM_PER_SECOND * 1000.0;
        driveRobot(Math.signum(cm) * DRIVE_POWER, 0, 0);
        sleep((long) timeMs);
        stopDrive();
    }

    void strafe(double cm) {
        double timeMs = Math.abs(cm) / CM_PER_SECOND * 1000.0;
        driveRobot(0, Math.signum(cm) * STRAFE_POWER, 0);
        sleep((long) timeMs);
        stopDrive();
    }

    void turnRelative(double deg) {
        double timeMs = Math.abs(deg) / DEG_PER_SECOND * 1000.0;
        driveRobot(0, 0, Math.signum(deg) * TURN_POWER);
        sleep((long) timeMs);
        stopDrive();
    }
    void driveRobot(double f, double s, double r) {
        drive.LUmotor.setPower(f + s + r);
        drive.LDmotor.setPower(f - s + r);
        drive.RUmotor.setPower(f - s - r);
        drive.RDmotor.setPower(f + s - r );
    }

    void stopDrive() {
        driveRobot(0, 0, 0);
    }
}