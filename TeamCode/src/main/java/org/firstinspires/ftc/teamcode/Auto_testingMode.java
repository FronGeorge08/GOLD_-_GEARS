package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="Auto_testing_mode", group="Linear OpMode")
public class Auto_testingMode extends LinearOpMode {

    // Drive motors
    DcMotor LUmotor, LDmotor, RUmotor, RDmotor;

    // Mechanisms
    DcMotorEx shooter;
    DcMotor intake;
    CRServo servoS, servoD;

    // Constants
    private static final double SHOOTER_POWER = 0.8;
    private static final double INTAKE_POWER = 1.0;

    private static final double SERVO_S_SHOOT = -1.0;
    private static final double SERVO_D_SHOOT = 1.0;
    private static final double SERVO_STOP = 0.0;

    private static final double FEED_TIME = 0.35;
    private static final double PAUSE_TIME = 0.45;

    // Shooter state
    private boolean shootingSequence = false;
    private boolean feeding = false;
    private int targetShots = 0;
    private int shotsDone = 0;
    private double stateStartTime = 0;

    // Drive constants
    private static final double WHEEL_DIAMETER_CM = 10.4; // 4 inch wheels
    private static final double TICKS_PER_REV = 537.6; // Encoder ticks
    private static final double GEAR_RATIO = 1.0; // 1:1
    private static final double CM_PER_TICK = (Math.PI * WHEEL_DIAMETER_CM) / (TICKS_PER_REV * GEAR_RATIO);

    @Override
    public void runOpMode() {

        // Initialize drive motors
        initDriveMotors();

        // Initialize mechanisms
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servoS = hardwareMap.get(CRServo.class, "servoS");
        servoD = hardwareMap.get(CRServo.class, "servoD");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        stopServos();

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        // ================= AUTO SEQUENCE =================

        // 1️⃣ Move forward 7.5 cm
        moveForwardCm(7.5, 0.5);

        // 2️⃣ Shoot 3 balls
        shooter.setPower(SHOOTER_POWER);
        startShooting(3);
        while (shootingSequence) {
            updateShooting();
        }
        shooter.setPower(0);

        // 3️⃣ Rotate 285 degrees clockwise
        rotateDegrees(285, 0.5);

        // 4️⃣ Move forward 5 cm
        moveForwardCm(5, 0.5);
    }

    // ================= DRIVE METHODS =================
    private void moveForwardCm(double cm, double power) {
        int ticks = (int) Math.round(cm / CM_PER_TICK);

        LUmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RUmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LUmotor.setTargetPosition(ticks);
        LDmotor.setTargetPosition(ticks);
        RUmotor.setTargetPosition(ticks);
        RDmotor.setTargetPosition(ticks);

        LUmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RUmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LUmotor.setPower(power);
        LDmotor.setPower(power);
        RUmotor.setPower(power);
        RDmotor.setPower(power);

        while (opModeIsActive() &&
                (LUmotor.isBusy() || LDmotor.isBusy() || RUmotor.isBusy() || RDmotor.isBusy())) {
            telemetry.addData("Moving forward", cm + " cm");
            telemetry.update();
        }

        stopDriveMotors();
    }

    private void rotateDegrees(double degrees, double power) {
        // Approximate: 1 degree ~ 10 cm wheel travel (adjust for your robot)
        double rotationCircumference = 35; // distance robot travels per 360° rotation in cm (adjust experimentally)
        double cm = (rotationCircumference * degrees) / 360.0;
        int ticks = (int) Math.round(cm / CM_PER_TICK);

        LUmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RUmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Clockwise rotation: left forward, right backward
        LUmotor.setTargetPosition(ticks);
        LDmotor.setTargetPosition(ticks);
        RUmotor.setTargetPosition(-ticks);
        RDmotor.setTargetPosition(-ticks);

        LUmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RUmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LUmotor.setPower(power);
        LDmotor.setPower(power);
        RUmotor.setPower(power);
        RDmotor.setPower(power);

        while (opModeIsActive() &&
                (LUmotor.isBusy() || LDmotor.isBusy() || RUmotor.isBusy() || RDmotor.isBusy())) {
            telemetry.addData("Rotating", degrees + " deg");
            telemetry.update();
        }

        stopDriveMotors();
    }

    private void stopDriveMotors() {
        LUmotor.setPower(0);
        LDmotor.setPower(0);
        RUmotor.setPower(0);
        RDmotor.setPower(0);
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
    }

    // ================= SHOOTING LOGIC =================
    private void startShooting(int shots) {
        shootingSequence = true;
        feeding = true;
        targetShots = shots;
        shotsDone = 0;
        stateStartTime = getRuntime();
    }

    private void updateShooting() {
        if (!shootingSequence) return;

        double elapsed = getRuntime() - stateStartTime;

        if (feeding && elapsed >= FEED_TIME) {
            feeding = false;
            shotsDone++;
            stateStartTime = getRuntime();
        }
        else if (!feeding && shotsDone < targetShots && elapsed >= PAUSE_TIME) {
            feeding = true;
            stateStartTime = getRuntime();
        }

        if (feeding) {
            servoS.setPower(-SERVO_S_SHOOT);
            servoD.setPower(-SERVO_D_SHOOT);
        } else {
            stopServos();
        }

        if (shotsDone >= targetShots) {
            shootingSequence = false;
            stopServos();
        }
    }

    private void stopServos() {
        servoS.setPower(SERVO_STOP);
        servoD.setPower(SERVO_STOP);
    }
}
