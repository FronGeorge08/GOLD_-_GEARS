package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto_7cm_Forward_3cm_Left", group = "Linear OpMode")
public class Auto_7cm_Forward_3cm_Left extends LinearOpMode {

    // Drive motors
    DcMotor LUmotor, LDmotor, RUmotor, RDmotor;
    // ===== CONSTANTS =====
    private static final double WHEEL_DIAMETER_CM = 10.4;   // 4 inch wheels
    private static final double TICKS_PER_REV = 537.6;     // GoBILDA 5202
    private static final double CM_PER_TICK =
            (Math.PI * WHEEL_DIAMETER_CM) / TICKS_PER_REV;

    @Override
    public void runOpMode() {

        initDriveMotors();

        telemetry.addLine("Autonomous Ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // 1️⃣ Move forward 7 cm
        moveCm(2, 0, 0.1);

        // 2️⃣ Move left 3 cm
        moveCm(0, -3, 0.1);

        // 3️⃣ Stop
        stopMotors();
    }

    // ================= MOVE METHOD =================
    private void moveCm(double forwardCm, double strafeCm, double power) {

        int forwardTicks = (int)(forwardCm / CM_PER_TICK);
        int strafeTicks  = (int)(strafeCm  / CM_PER_TICK);

        LUmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RUmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Mecanum encoder targets
        LUmotor.setTargetPosition(forwardTicks + strafeTicks);
        LDmotor.setTargetPosition(forwardTicks - strafeTicks);
        RUmotor.setTargetPosition(forwardTicks - strafeTicks);
        RDmotor.setTargetPosition(forwardTicks + strafeTicks);

        LUmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RUmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LUmotor.setPower(power);
        LDmotor.setPower(power);
        RUmotor.setPower(power);
        RDmotor.setPower(power);

        while (opModeIsActive() &&
                (LUmotor.isBusy() || LDmotor.isBusy() ||
                        RUmotor.isBusy() || RDmotor.isBusy())) {
            idle();
        }

        stopMotors();

        // Return to normal mode
        LUmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RUmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ================= HELPERS =================
    private void stopMotors() {
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

        // Reverse left side (standard mecanum setup)
        LUmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LDmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
