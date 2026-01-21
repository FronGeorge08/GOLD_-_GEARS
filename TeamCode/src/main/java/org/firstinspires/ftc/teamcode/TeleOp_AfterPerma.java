package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp_ServoShoot_Toggle", group="Linear OpMode")
public class TeleOp_AfterPerma extends LinearOpMode {

    // Drive motors
    DcMotor LUmotor, LDmotor, RUmotor, RDmotor;

    // Shooter and intake
    DcMotorEx shooter;
    DcMotor intake;

    // Positional servo
    Servo feederServo;

    // Shooter constants
    private static final double SHOOTER_POWER = 0.8;
    private static final double INTAKE_POWER  = 0.75;

    // Servo angles
    private static final double SERVO_START_DEG = -90;  // initial angle
    private static final double SERVO_SHOOT_DEG = 90;   // shoot angle

    private static final double FEED_TIME = 0.35;
    private static final double PAUSE_TIME = 0.45;

    // Servo state
    private boolean shootingSequence = false;
    private boolean feeding = false;
    private int targetShots = 0;
    private int shotsDone = 0;
    private double stateStartTime = 0;

    // Toggles
    private boolean intakeOn = false;
    private boolean shooterOn = false;

    // Gamepad previous states
    private boolean aLast = false;
    private boolean yLast = false;
    private boolean rtLast = false;
    private boolean ltLast = false;

    @Override
    public void runOpMode() {

        // Initialize drive
        initDriveMotors();

        // Initialize hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        feederServo = hardwareMap.get(Servo.class, "servoS"); // single positional servo

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Start servo at initial angle
        setServoAngle(SERVO_START_DEG);

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ---------- Drive ----------
            driveMecanum();

            // ---------- Intake toggle (A) ----------
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) intakeOn = !intakeOn;
            aLast = aNow;
            intake.setPower(intakeOn ? INTAKE_POWER : 0);

            // ---------- Shooter toggle (Y) ----------
            boolean yNow = gamepad1.y;
            if (yNow && !yLast) shooterOn = !shooterOn;
            yLast = yNow;
            shooter.setPower(shooterOn ? SHOOTER_POWER : 0);

            // ---------- Shooting ----------
            boolean rtNow = gamepad1.right_trigger > 0.5;
            if (rtNow && !rtLast && shooterOn && !shootingSequence) startShooting(1);
            rtLast = rtNow;

            boolean ltNow = gamepad1.left_trigger > 0.5;
            if (ltNow && !ltLast && shooterOn && !shootingSequence) startShooting(2);
            ltLast = ltNow;

            updateShooting();

            telemetry.addData("Shooter", shooterOn);
            telemetry.addData("Intake", intakeOn);
            telemetry.addData("Shots", shotsDone + "/" + targetShots);
            telemetry.update();
        }
    }

    // ------------------- Shooting Logic -------------------
    private void startShooting(int shots) {
        shootingSequence = true;
        feeding = true;
        targetShots = shots;
        shotsDone = 0;
        stateStartTime = getRuntime();
        // Move servo to shooting angle
        setServoAngle(SERVO_SHOOT_DEG);
    }

    private void updateShooting() {
        if (!shootingSequence) return;

        double elapsed = getRuntime() - stateStartTime;

        if (feeding && elapsed >= FEED_TIME) {
            feeding = false;
            shotsDone++;
            stateStartTime = getRuntime();
        } else if (!feeding && shotsDone < targetShots && elapsed >= PAUSE_TIME) {
            feeding = true;
            stateStartTime = getRuntime();
        }

        if (shotsDone >= targetShots) {
            shootingSequence = false;
            // Return servo to start angle
            setServoAngle(SERVO_START_DEG);
        }
    }

    // ------------------- Helpers -------------------
    private void setServoAngle(double degrees) {
        // Map -90°→90° to 0.0→1.0 for servo.setPosition()
        double position = (degrees + 90.0) / 180.0; // -90° => 0.0, 0° => 0.5, 90° => 1.0
        feederServo.setPosition(position);
    }

    private void driveMecanum() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = Math.abs(gamepad1.right_stick_x) > 0.1 ? gamepad1.right_stick_x : 0;

        double LU = y + x + rx;
        double LD = y - x + rx;
        double RU = y - x - rx;
        double RD = y + x - rx;

        double max = Math.max(Math.abs(LU), Math.max(Math.abs(LD), Math.max(Math.abs(RU), Math.abs(RD))));
        if (max > 1.0) { LU /= max; LD /= max; RU /= max; RD /= max; }

        LUmotor.setPower(LU);
        LDmotor.setPower(LD);
        RUmotor.setPower(RU);
        RDmotor.setPower(RD);
    }

    private void initDriveMotors() {
        LUmotor = hardwareMap.get(DcMotor.class, "LUmotor");
        LDmotor = hardwareMap.get(DcMotor.class, "LDmotor");
        RUmotor = hardwareMap.get(DcMotor.class, "RUmotor");
        RDmotor = hardwareMap.get(DcMotor.class, "RDmotor");

        LUmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LDmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LUmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RUmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
