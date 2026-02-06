package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "DECODE_TELEOPwqd", group = "Linear OpMode")
@Config
@Disabled
public class TeleOp_AfterPerma_testingMode extends LinearOpMode {
    HardWare robot;
    public static double SHOOTER_POWER = 0.4;//0.89 far
    public static double INTAKE_POWER = 0.80;
    public static double FEED_POWER = 1.0;
    public static double FEED_STOP = 0.0;

    private boolean intakeOn = false;
    private boolean shooterOn = false;

    private boolean aLast = false;
    private boolean yLast = false;

    @Override
    public void runOpMode() {

        robot = new HardWare(hardwareMap);

        stopFeed();
        robot.servo_shooter_cover.setPosition(1);
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

            boolean shootNow =
                    gamepad1.right_bumper ||
                            gamepad1.left_bumper;

            double intakePower;

            if (shootNow && shooterOn) {
                robot.servo_shooter_cover.setPosition(0);
                intakePower = 0.9;
                robot.servoS.setPower(FEED_POWER);
                robot.servoR.setPower(FEED_POWER);
                sleep(500);
                robot.servo_shooter_cover.setPosition(1);
            } else {
                intakePower = intakeOn ? INTAKE_POWER : 0;
                stopFeed();
            }

            robot.intake.setPower(intakePower);

            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("Shooter Power", shooterOn ? SHOOTER_POWER : 0);
            telemetry.addData("Intake ON", intakeOn);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Feeding", shootNow && shooterOn);
            telemetry.update();
        }
    }


    private void stopFeed() {
        robot.servoS.setPower(FEED_STOP);
        robot.servoR.setPower(FEED_STOP);
    }

    private void driveMecanum() {

        double y = -gamepad1.left_stick_y;
        double x =gamepad1.left_stick_x;
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