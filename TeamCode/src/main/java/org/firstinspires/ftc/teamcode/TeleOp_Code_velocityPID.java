package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.hardware.ShooterHardware;

@TeleOp(name = "DECODE_TELEOP", group = "Linear OpMode")
@Config
public class TeleOp_Code_velocityPID extends LinearOpMode {

    DriveHardware drive;
    ShooterHardware shooterHw;

    public static double SHOOTER_POWER = 0.78;
    public static double SHOOTER_POWER_FAR = 0.89;
    public static double INTAKE_POWER = 0.7;
    public static double FEED_POWER = 1.0;
    public static double COVER_OPEN_TIME = 0.5;
    public static double COVER_CLOSE_TIME = 0.4;
    public static double SHOOTER_RECOVER_TIME = 1.6;
    public static double INTAKE_SHOOTING = 0.5;
    private boolean intakeOn = false;
    private boolean shooterOn = false;
    private boolean shooterPowerFarMode = false;
    private boolean aLast = false;
    private boolean yLast = false;
    private boolean rbLast = false;
    private boolean lbLast = false;

    private boolean shootingActive = false;
    private int shotsRemaining = 0;

    private double shooterPowerAdjustment = 0;

    ElapsedTime shotTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        drive = new DriveHardware(hardwareMap);
        shooterHw = new ShooterHardware(hardwareMap);
        stopFeed();
        shooterHw.shooterCover.setPosition(0.9);

        double voltage = 0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltage = Math.max(voltage, sensor.getVoltage());
        }

        if (voltage <= 13.20 && voltage > 12.80) {
            shooterPowerAdjustment = -0.04;
        } else if (voltage <= 12.80 && voltage > 12.40) {
            shooterPowerAdjustment = 0;
        } else if (voltage <= 12.40 && voltage > 12.0) {
            shooterPowerAdjustment = 0.02;
        } else if (voltage <= 12.0) {
            shooterPowerAdjustment = 0.03;
        }

        telemetry.addData("Battery Voltage", voltage);
        telemetry.addData("Shooter Power Adjustment", shooterPowerAdjustment);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            driveMecanum();

            boolean aNow = gamepad1.a;
            if (aNow && !aLast) intakeOn = !intakeOn;
            aLast = aNow;

            boolean yNow = gamepad1.y;
            if (yNow && !yLast) {
                shooterOn = !shooterOn;

                if (shooterOn) {
                    gamepad1.rumble(200);
                } else {
                    gamepad1.stopRumble();
                }
            }
            yLast = yNow;

            if (gamepad1.dpad_up) {
                shooterPowerFarMode = true;
            }
            if (gamepad1.dpad_down) {
                shooterPowerFarMode = false;
            }
            double basePower = shooterPowerFarMode ? SHOOTER_POWER_FAR : SHOOTER_POWER;
            double currentShooterPower = basePower + shooterPowerAdjustment;
            shooterHw.shooter.setPower(shooterOn ? currentShooterPower : 0);

            if (shooterOn) {
                double rumblePower = shooterPowerFarMode ? 0.7 : 0.2;

                if (!gamepad1.isRumbling()) {
                    gamepad1.rumble(rumblePower, rumblePower, 500);
                }
            } else {
                gamepad1.stopRumble();
            }

            boolean rbNow = gamepad1.right_bumper;
            boolean lbNow = gamepad1.left_bumper;

            if (rbNow && !rbLast && shooterOn && !shootingActive) {
                shotsRemaining = 1;
                startShooting();
            }

            if (lbNow && !lbLast && shooterOn && !shootingActive) {
                shotsRemaining = 3;
                startShooting();
            }

            rbLast = rbNow;
            lbLast = lbNow;

            if (shootingActive) {

                double t = shotTimer.seconds();

                double openEnd = COVER_OPEN_TIME;
                double closeEnd = openEnd + COVER_CLOSE_TIME;
                double recoverEnd = closeEnd +
                        ((shotsRemaining > 1) ? SHOOTER_RECOVER_TIME : 0);

                if (t < openEnd) {
                    shooterHw.shooterCover.setPosition(0);
                    shooterHw.servoS.setPower(FEED_POWER);
                    shooterHw.servoR.setPower(FEED_POWER);
                    shooterHw.intake.setPower(INTAKE_SHOOTING);
                }
                else if (t < closeEnd) {
                    shooterHw.shooterCover.setPosition(0.9);
                    stopFeed();
                    shooterHw.intake.setPower(INTAKE_SHOOTING);
                }
                else if (t < recoverEnd) {
                    shooterHw.shooterCover.setPosition(0.9);
                    stopFeed();
                    shooterHw.intake.setPower(INTAKE_SHOOTING);
                }
                else {
                    shotsRemaining--;
                    if (shotsRemaining > 0) {
                        shotTimer.reset();
                    } else {
                        shootingActive = false;
                        shooterHw.shooterCover.setPosition(0.9);
                        stopFeed();
                    }
                }
            }
            else {
                if (gamepad1.b) {
                    shooterHw.intake.setPower(-1.0);
                } else {
                    shooterHw.intake.setPower(intakeOn ? INTAKE_POWER : 0);
                }
            }

            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("Shooter Power Mode", shooterPowerFarMode ? "FAR" : "CLOSE");
            telemetry.addData("Adjusted Shooter Power", currentShooterPower);
            telemetry.addData("Shooting", shootingActive);
            telemetry.addData("Shots Left", shotsRemaining);
            telemetry.addData("Intake Forward", intakeOn);
            telemetry.addData("Intake Reverse (B)", gamepad1.b);
            telemetry.update();
        }
    }
    private void startShooting() {
        shootingActive = true;
        shotTimer.reset();
    }
    private void stopFeed() {
        shooterHw.servoS.setPower(0);
        shooterHw.servoR.setPower(0);
    }

    private void driveMecanum() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;

        double LU = y + x + rx;
        double LD = y - x + rx;
        double RU = y - x - rx;
        double RD = y + x - rx;

        double max = Math.max(
                Math.abs(LU),
                Math.max(Math.abs(LD), Math.max(Math.abs(RU), Math.abs(RD)))
        );

        if (max > 1.0) {
            LU /= max;
            LD /= max;
            RU /= max;
            RD /= max;
        }

        drive.LUmotor.setPower(LU);
        drive.LDmotor.setPower(LD);
        drive.RUmotor.setPower(RU);
        drive.RDmotor.setPower(RD);
    }
}