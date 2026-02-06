package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ODOMETRY_TEST_ODOMETRY", group = "Test")
@Config
@Disabled
public class MecanumDriveTeleOp extends LinearOpMode {

    HardWare robot;
    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_DIAMETER_CM = 3.5;
    public static double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_CM / 2.54;
    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_IN;
    public static double TICKS_TO_INCH = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
    int lastXTicks = 0;
    int lastYTicks = 0;
    double xPos = 0;
    double yPos = 0;

    @Override
    public void runOpMode() {

        robot = new HardWare(hardwareMap);

        robot.LDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // X
        robot.RDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Y

        robot.LDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RDmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("ODOMETRY TEST READY");
        telemetry.addLine("Drive robot normally");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            driveTeleOp();

            int xTicks = robot.LDmotor.getCurrentPosition();
            int yTicks = robot.RDmotor.getCurrentPosition();

            int deltaX = xTicks - lastXTicks;
            int deltaY = yTicks - lastYTicks;

            lastXTicks = xTicks;
            lastYTicks = yTicks;

            double dx = deltaX * TICKS_TO_INCH;
            double dy = deltaY * TICKS_TO_INCH;

            xPos += dx;
            yPos += dy;

            telemetry.addLine("=== RAW ENCODERS ===");
            telemetry.addData("X Encoder (LD)", xTicks);
            telemetry.addData("Y Encoder (RD)", yTicks);

            telemetry.addLine("");
            telemetry.addLine("=== POSITION (inches) ===");
            telemetry.addData("X", xPos);
            telemetry.addData("Y", yPos);

            telemetry.update();
        }
    }

    private void driveTeleOp() {

        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double rotate = gamepad1.right_trigger - gamepad1.left_trigger;

        driveRobot(forward, strafe, rotate);
    }

    private void driveRobot(double forward, double strafe, double rotate) {

        double LU = forward + strafe + rotate;
        double LD = forward - strafe + rotate;
        double RU = forward - strafe - rotate;
        double RD = forward + strafe - rotate;

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
