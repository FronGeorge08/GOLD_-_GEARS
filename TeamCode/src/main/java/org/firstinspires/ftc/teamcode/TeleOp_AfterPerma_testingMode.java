package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="DECODE_TELEOP_TESTING_MODE", group="Linear OpMode")
@Config
public class TeleOp_AfterPerma_testingMode extends LinearOpMode {

    // ================= DRIVE =================
    DcMotor LUmotor, LDmotor, RUmotor, RDmotor;

    // ================= MECHANISMS =================
    DcMotorEx shooter;
    DcMotor intake;
    CRServo servoS; // LEFT
    CRServo servoR; // RIGHT

    // ================= CONSTANTS =================
    private static double SHOOTER_POWER = 0.95;
    private static double INTAKE_POWER  = 0.45;

    private static double SERVO_SHOOT =  1.0;
    private static double SERVO_STOP  =  0.0;

    // ================= STATE =================
    private boolean intakeOn  = false;
    private boolean shooterOn = false;

    private boolean aLast = false;
    private boolean yLast = false;

    @Override
    public void runOpMode() {

        // -------- DRIVE INIT --------
        initDriveMotors();

        // -------- MECHANISMS INIT --------
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake  = hardwareMap.get(DcMotor.class, "intake");

        servoS = hardwareMap.get(CRServo.class, "servoS");
        servoR = hardwareMap.get(CRServo.class, "servoR");

        // Reverse ONE servo if physically mirrored
        servoR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopServos();

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        // ================= LOOP =================
        while (opModeIsActive()) {

            // -------- DRIVE --------
            driveMecanum();

            // -------- INTAKE TOGGLE (A) --------
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) intakeOn = !intakeOn;
            aLast = aNow;

            intake.setPower(intakeOn ? INTAKE_POWER : 0);

            // -------- SHOOTER TOGGLE (Y) --------
            boolean yNow = gamepad1.y;
            if (yNow && !yLast) shooterOn = !shooterOn;
            yLast = yNow;

            shooter.setPower(shooterOn ? SHOOTER_POWER : 0);

            // -------- SHOOT BUTTONS --------
            boolean shootNow =
                    (gamepad1.right_trigger > 0.5) ||
                            (gamepad1.left_trigger  > 0.5);

            // -------- SERVO LOGIC (SHOOT ONLY) --------
            if (shootNow && shooterOn) {
                intake.setPower(1);
                servoS.setPower(SERVO_SHOOT);
                servoR.setPower(SERVO_SHOOT);
            } else {
                intake.setPower(0);
                stopServos();
            }

            // -------- TELEMETRY --------
            telemetry.addData("Intake", intakeOn);
            telemetry.addData("Shooter", shooterOn);
            telemetry.addData("Shooting", shootNow);
            telemetry.update();
        }
    }

    // ================= HELPERS =================
    private void stopServos() {
        servoS.setPower(SERVO_STOP);
        servoR.setPower(SERVO_STOP);
    }

    private void driveMecanum() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double LU = y + x + rx;
        double LD = y - x + rx;
        double RU = y - x - rx;
        double RD = y + x - rx;

        double max = Math.max(Math.abs(LU),
                Math.max(Math.abs(LD), Math.max(Math.abs(RU), Math.abs(RD))));

        if (max > 1.0) {
            LU /= max;
            LD /= max;
            RU /= max;
            RD /= max;
        }

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

        LUmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RUmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
