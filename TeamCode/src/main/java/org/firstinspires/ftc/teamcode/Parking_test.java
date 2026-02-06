package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "Parking_test")
@Disabled
public class Parking_test extends LinearOpMode {
    DcMotor motor;
    ElapsedTime timer = new ElapsedTime();
    boolean motorRunning = false;
    boolean lastAState = false;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "keeper");
        waitForStart();
        while (opModeIsActive()) {
            boolean currentAState = gamepad1.a;
            if (currentAState && !lastAState && !motorRunning) {
                motorRunning = true;
                timer.reset();
            }
            if (motorRunning) {
                motor.setPower(1);
                if (timer.seconds() >= 2.0) {
                    motor.setPower(0);
                    motorRunning = false;
                }
            }
            lastAState = currentAState;
            telemetry.addData("Motor Running", motorRunning);
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
        }
    }
}
