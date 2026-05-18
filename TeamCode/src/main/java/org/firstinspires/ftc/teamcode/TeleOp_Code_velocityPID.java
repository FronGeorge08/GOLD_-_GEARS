package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.hardware.ShooterHardware;

@TeleOp(name = "DECODE_TELEOP", group = "Linear OpMode")
@Config
public class TeleOp_Code_velocityPID extends LinearOpMode {

    // ---------------- HARDWARE ----------------

    private DriveHardware drive;
    private ShooterHardware shooterHw;

    // ---------------- SHOOTER CONFIG ----------------

    // Tune these FIRST
    public static double SHOOTER_VELOCITY_CLOSE = 1800;

    public static double SHOOTER_VELOCITY_FAR = 2100;

    // Feed tuning
    public static double FEED_POWER = 1.0;

    public static double FEED_TIME = 0.16;

    // Velocity tolerance for recovery
    public static double RECOVERY_TOLERANCE = 120;

    // ---------------- INTAKE CONFIG ----------------

    public static double INTAKE_POWER = 0.75;

    public static double INTAKE_SHOOTING = 0.45;

    // ---------------- SHOOTER STATES ----------------

    enum ShootState {
        IDLE,
        FEED,
        RECOVER
    }

    private ShootState shootState = ShootState.IDLE;

    // ---------------- FLAGS ----------------

    private boolean intakeOn = false;

    private boolean shooterOn = false;

    private boolean farMode = false;

    // ---------------- BUTTON CACHE ----------------

    private boolean aLast = false;

    private boolean yLast = false;

    private boolean rbLast = false;

    // ---------------- SHOOTER CONTROL ----------------

    private int shotsRemaining = 0;

    private final ElapsedTime shotTimer = new ElapsedTime();

    // Optimization:
    // prevents spamming setVelocity every loop
    private double lastTargetVelocity = 0;

    @Override
    public void runOpMode() {

        drive = new DriveHardware(hardwareMap);

        shooterHw = new ShooterHardware(hardwareMap);

        shooterHw.closeCover();

        shooterHw.stopFeed();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive()) {

            // =========================================================
            // DRIVE
            // =========================================================

            double y = -gamepad1.left_stick_y;

            double x = gamepad1.left_stick_x;

            double rx =
                    gamepad1.right_trigger
                            - gamepad1.left_trigger;

            drive.driveMecanum(y, x, rx);

            // =========================================================
            // INTAKE TOGGLE
            // =========================================================

            boolean aNow = gamepad1.a;

            if (aNow && !aLast) {

                intakeOn = !intakeOn;
            }

            aLast = aNow;

            // =========================================================
            // SHOOTER TOGGLE
            // =========================================================

            boolean yNow = gamepad1.y;

            if (yNow && !yLast) {

                shooterOn = !shooterOn;

                if (!shooterOn) {

                    shootState = ShootState.IDLE;

                    shooterHw.stopFeed();

                    shooterHw.closeCover();

                    lastTargetVelocity = 0;
                }
            }

            yLast = yNow;

            // =========================================================
            // DISTANCE MODE
            // =========================================================

            if (gamepad1.dpad_up) {

                farMode = true;
            }

            if (gamepad1.dpad_down) {

                farMode = false;
            }

            // =========================================================
            // TARGET VELOCITY
            // =========================================================

            double targetVelocity =
                    farMode
                            ? SHOOTER_VELOCITY_FAR
                            : SHOOTER_VELOCITY_CLOSE;

            // =========================================================
            // SHOOTER MOTOR CONTROL
            // =========================================================

            if (shooterOn) {

                if (targetVelocity != lastTargetVelocity) {

                    shooterHw.shooter.setVelocity(targetVelocity);

                    lastTargetVelocity = targetVelocity;
                }

            } else {

                shooterHw.shooter.setPower(0);

                lastTargetVelocity = 0;
            }

            // =========================================================
            // CURRENT VELOCITY
            // =========================================================

            double currentVelocity =
                    shooterHw.shooter.getVelocity();

            boolean shooterReady =
                    Math.abs(
                            currentVelocity - targetVelocity
                    ) < RECOVERY_TOLERANCE;

            // =========================================================
            // TRIPLE SHOT TRIGGER
            // =========================================================

            boolean rbNow = gamepad1.right_bumper;

            if (rbNow
                    && !rbLast
                    && shooterOn
                    && shootState == ShootState.IDLE) {

                shotsRemaining = 3;

                // IMPORTANT:
                // Wait for velocity before first shot

                if (shooterReady) {

                    shootState = ShootState.FEED;

                } else {

                    shootState = ShootState.RECOVER;
                }

                shotTimer.reset();
            }

            rbLast = rbNow;

            // =========================================================
            // SHOOTER STATE MACHINE
            // =========================================================

            switch (shootState) {

                // -----------------------------------------------------

                case IDLE:

                    shooterHw.stopFeed();

                    shooterHw.closeCover();

                    break;

                // -----------------------------------------------------

                case FEED:

                    shooterHw.openCover();

                    shooterHw.servoS.setPower(FEED_POWER);

                    shooterHw.servoR.setPower(FEED_POWER);

                    shooterHw.intake.setPower(INTAKE_SHOOTING);

                    if (shotTimer.seconds() > FEED_TIME) {

                        shooterHw.stopFeed();

                        shooterHw.closeCover();

                        shotsRemaining--;

                        shotTimer.reset();

                        shootState = ShootState.RECOVER;
                    }

                    break;

                // -----------------------------------------------------

                case RECOVER:

                    shooterHw.stopFeed();

                    shooterHw.closeCover();

                    if (shotsRemaining <= 0) {

                        shootState = ShootState.IDLE;

                    } else if (shooterReady) {

                        shotTimer.reset();

                        shootState = ShootState.FEED;
                    }

                    break;
            }

            // =========================================================
            // NORMAL INTAKE CONTROL
            // =========================================================

            if (shootState == ShootState.IDLE) {

                if (gamepad1.b) {

                    shooterHw.intake.setPower(-1.0);

                } else {

                    shooterHw.intake.setPower(
                            intakeOn ? INTAKE_POWER : 0
                    );
                }
            }

            // =========================================================
            // TELEMETRY
            // =========================================================

            telemetry.addData(
                    "Shooter ON",
                    shooterOn
            );

            telemetry.addData(
                    "Far Mode",
                    farMode
            );

            telemetry.addData(
                    "Shoot State",
                    shootState
            );

            telemetry.addData(
                    "Shots Remaining",
                    shotsRemaining
            );

            telemetry.addData(
                    "Target Velocity",
                    targetVelocity
            );

            telemetry.addData(
                    "Current Velocity",
                    currentVelocity
            );

            telemetry.addData(
                    "Velocity Error",
                    targetVelocity - currentVelocity
            );

            telemetry.addData(
                    "Shooter Ready",
                    shooterReady
            );

            telemetry.addData(
                    "Intake ON",
                    intakeOn
            );

            telemetry.update();
        }
    }
}