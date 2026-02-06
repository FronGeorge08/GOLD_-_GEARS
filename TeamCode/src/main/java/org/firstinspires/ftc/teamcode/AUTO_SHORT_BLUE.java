package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.hardware.ShooterHardware;

@Autonomous(name = "AUTO_SHORT_BLUE", group = "AUTO")
@Config
public class AUTO_SHORT_BLUE extends LinearOpMode {

    DriveHardware drive;
    ShooterHardware shooter;
    public static double DRIVE_POWER  = 0.54;
    public static double STRAFE_POWER = 0.6;
    public static double TURN_POWER   = 0.5;

    public static double CM_PER_SECOND  = 55.0;
    public static double DEG_PER_SECOND = 120.0;

    public static double RAMP_TIME_MS = 150;
    public static double MIN_POWER = 0.25;

    public static double LEFT_SCALE  = 1.0;
    public static double RIGHT_SCALE = 0.96;
    public static double L = 88;
    public static double D = 40;
    public static double SHOOTER_POWER = 0.78;
    public static double FEED_POWER = 1.0;
    public static double COVER_OPEN_TIME = 0.4;
    public static double COVER_CLOSE_TIME = 0.3;
    public static double SHOOTER_RECOVER_TIME = 2.1;
    public static double INTAKE_SHOOTING = 0.7;
    public static double LITTLE_ANGLE_SWITCH = 4;
    private boolean shootingActive = false;
    private int shotsRemaining = 0;
    private ElapsedTime shotTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        drive = new DriveHardware(hardwareMap);
        shooter = new ShooterHardware(hardwareMap);

        shooter.shooterCover.setPosition(0.9);
        stopFeed();

        double voltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, sensor.getVoltage());
        }

        if (voltage >= 13.30) {
            L = 79;
            D = 33;
            SHOOTER_POWER = 0.72;
            DRIVE_POWER = 0.54;
            LITTLE_ANGLE_SWITCH=5;
        }
        else if (voltage >= 13.00) {
            L = 80;
            D = 36;
            SHOOTER_POWER = 0.74;
            DRIVE_POWER = 0.54;
            LITTLE_ANGLE_SWITCH=4;
        }
        else if (voltage >= 12.70) {
            L = 82;
            D = 36;
            SHOOTER_POWER = 0.76;
            DRIVE_POWER = 0.555;
            LITTLE_ANGLE_SWITCH=4;
        }
        else if (voltage >= 12.50) {
            L = 84;
            D = 38;
            SHOOTER_POWER = 0.78;
            DRIVE_POWER = 0.55;
            LITTLE_ANGLE_SWITCH=4;
        }
        else if (voltage >= 12.30) {
            L = 86;
            D = 39;
            SHOOTER_POWER = 0.80;
            DRIVE_POWER = 0.56;
            LITTLE_ANGLE_SWITCH=4;
        }
        else {
            L = 88;
            D = 40;
            SHOOTER_POWER = 0.82;
            DRIVE_POWER = 0.56;
            LITTLE_ANGLE_SWITCH=4;
        }

        telemetry.addLine("AUTO READY (VOLTAGE PROFILE)");
        telemetry.addData("Battery Voltage", voltage);
        telemetry.addData("L", L);
        telemetry.addData("D", D);
        telemetry.addData("Shooter Power", SHOOTER_POWER);
        telemetry.addData("Drive Power", DRIVE_POWER);
        telemetry.update();

        waitForStart();

        shooter.shooter.setPower(SHOOTER_POWER);
        shooter.intake.setPower(INTAKE_SHOOTING);
        sleep(1600);

        driveForwardCM(-L - 8);
        turnRelative(-LITTLE_ANGLE_SWITCH - 5);
        sleep(600);
        shoot3();

        shooter.intake.setPower(0.7);
        turnRelative(-D - LITTLE_ANGLE_SWITCH - 5);
        driveForwardCM(L-8);
        driveForwardCM(-(L-13));
        turnRelative(D + LITTLE_ANGLE_SWITCH - 4);
        sleep(200);
        shoot3();

        turnRelative(-D - LITTLE_ANGLE_SWITCH+5);
        strafeLeftCM(-25);


        shooter.shooter.setPower(0);
        shooter.intake.setPower(0);
        stopDrive();
    }

    private void driveForwardCM(double cm) {
        runTimedMotion(Math.signum(cm) * DRIVE_POWER, 0, 0,
                Math.abs(cm) / CM_PER_SECOND * 1000);
    }

    private void strafeLeftCM(double cm) {
        runTimedMotion(0, Math.signum(cm) * STRAFE_POWER, 0,
                Math.abs(cm) / CM_PER_SECOND * 1000);
    }

    private void turnRelative(double deg) {
        runTimedMotion(0, 0, Math.signum(deg) * TURN_POWER,
                Math.abs(deg) / DEG_PER_SECOND * 1000);
    }

    private void runTimedMotion(double f, double s, double r, double durationMs) {
        long start = System.currentTimeMillis();

        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            double elapsed = System.currentTimeMillis() - start;
            double ramp = Math.min(elapsed / RAMP_TIME_MS, 1.0);
            double powerMultiplier = Math.max(ramp, MIN_POWER);

            driveRobot(
                    f * powerMultiplier,
                    s * powerMultiplier,
                    r * powerMultiplier
            );
            idle();
        }
        stopDrive();
    }

    private void driveRobot(double f, double s, double r) {
        double lu = f + s + r;
        double ld = f - s + r;
        double ru = f - s - r;
        double rd = f + s - r;

        drive.LUmotor.setPower(lu * LEFT_SCALE);
        drive.LDmotor.setPower(ld * LEFT_SCALE);
        drive.RUmotor.setPower(ru * RIGHT_SCALE);
        drive.RDmotor.setPower(rd * RIGHT_SCALE);
    }

    private void stopDrive() {
        driveRobot(0, 0, 0);
    }
    private void shoot3() {
        shootN(3);
    }

    private void shootN(int shots) {
        shotsRemaining = shots;
        shootingActive = true;
        shotTimer.reset();

        while (opModeIsActive() && shootingActive) {
            double t = shotTimer.seconds();

            double openEnd = COVER_OPEN_TIME;
            double closeEnd = openEnd + COVER_CLOSE_TIME;
            double recoverEnd = closeEnd +
                    ((shotsRemaining > 1) ? SHOOTER_RECOVER_TIME : 0);

            if (t < openEnd) {
                shooter.shooterCover.setPosition(0);
                shooter.servoS.setPower(FEED_POWER);
                shooter.servoR.setPower(FEED_POWER);
                shooter.intake.setPower(INTAKE_SHOOTING);
            }
            else if (t < closeEnd) {
                shooter.shooterCover.setPosition(0.9);
                stopFeed();
                shooter.intake.setPower(INTAKE_SHOOTING);
            }
            else if (t < recoverEnd) {
                shooter.shooterCover.setPosition(0.9);
                stopFeed();
                shooter.intake.setPower(INTAKE_SHOOTING);
            }
            else {
                shotsRemaining--;
                if (shotsRemaining > 0) shotTimer.reset();
                else shootingActive = false;
            }
        }

        stopFeed();
        shooter.intake.setPower(0);
        shooter.shooterCover.setPosition(0.9);
    }

    private void stopFeed() {
        shooter.servoS.setPower(0);
        shooter.servoR.setPower(0);
    }
}
