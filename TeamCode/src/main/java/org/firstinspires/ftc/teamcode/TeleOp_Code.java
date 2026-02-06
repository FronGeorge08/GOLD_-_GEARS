package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DECODE_TELEOP_TESTING_NEW_SHOOTING", group = "Linear OpMode")
@Config
@Disabled
public class TeleOp_Code extends LinearOpMode {

    HardWare robot;

    public static double SHOOTER_POWER = 0.81;
    public static double INTAKE_POWER = 0.7;
    public static double FEED_POWER = 1.0;
    public static double COVER_OPEN_TIME = 0.6;
    public static double COVER_CLOSE_TIME = 0.5;
    public static double SHOOTER_RECOVER_TIME = 1.6;
    public static double INTAKE_SHOOTING=0.5;

    private boolean intakeOn = false;
    private boolean shooterOn = false;

    private boolean aLast = false;
    private boolean yLast = false;
    private boolean rbLast = false;
    private boolean lbLast = false;

    private boolean shootingActive = false;
    private int shotsRemaining = 0;

    ElapsedTime shotTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot = new HardWare(hardwareMap);
        stopFeed();
        robot.servo_shooter_cover.setPosition(0.9);

        telemetry.addLine("TELEOP READY");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            driveMecanum();
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) intakeOn = !intakeOn;
            aLast = aNow;

            boolean yNow = gamepad1.y;
            if (yNow && !yLast) shooterOn = !shooterOn;
            yLast = yNow;

            robot.shooter.setPower(shooterOn ? SHOOTER_POWER : 0);

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
                    robot.servo_shooter_cover.setPosition(0);
                    robot.servoS.setPower(FEED_POWER);
                    robot.servoR.setPower(FEED_POWER);
                    robot.intake.setPower(INTAKE_SHOOTING);
                }
                else if (t < closeEnd) {
                    robot.servo_shooter_cover.setPosition(0.9);
                    stopFeed();
                    robot.intake.setPower(INTAKE_SHOOTING);
                }
                else if (t < recoverEnd) {
                    robot.servo_shooter_cover.setPosition(0.9);
                    stopFeed();
                    robot.intake.setPower(INTAKE_SHOOTING);
                }
                else {
                    shotsRemaining--;
                    if (shotsRemaining > 0) {
                        shotTimer.reset();
                    } else {
                        shootingActive = false;
                        robot.servo_shooter_cover.setPosition(0.9);
                        stopFeed();
                    }
                }
            }
            else {
                if (gamepad1.b) {
                    robot.intake.setPower(-1.0);
                } else {
                    robot.intake.setPower(intakeOn ? INTAKE_POWER : 0);
                }
            }

            telemetry.addData("Shooter ON", shooterOn);
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
        robot.servoS.setPower(0);
        robot.servoR.setPower(0);
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

        robot.LUmotor.setPower(LU);
        robot.LDmotor.setPower(LD);
        robot.RUmotor.setPower(RU);
        robot.RDmotor.setPower(RD);
    }
}
