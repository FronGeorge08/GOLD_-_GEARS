package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="DriverFocusedMecanum", group="Linear OpMode")
public class MecanumDriveTeleOp extends LinearOpMode {

    DcMotor LUmotor;
    DcMotor LDmotor;
    DcMotor RUmotor;
    DcMotor RDmotor;

    @Override
    public void runOpMode() {

        init_motors();

        telemetry.addLine("Driver-Focused Mecanum Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;

            double rxInput = gamepad1.right_stick_x;
            double rx = 0;

            double deadzone = 0.1;

            if (rxInput < -deadzone)
                rx = rxInput;
            else if (rxInput > deadzone)
                rx = rxInput;
            else
                rx = 0;

            double LU = y + x + rx;
            double LD = y - x + rx;
            double RU = y - x - rx;
            double RD = y + x - rx;

            double max = Math.max(Math.abs(LU), Math.max(Math.abs(LD),
                    Math.max(Math.abs(RU), Math.abs(RD))));

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
    }

    private void init_motors() {

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
