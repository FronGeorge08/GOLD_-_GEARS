package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "StarterBotTeleopTest")
public class TeleopTEST extends LinearOpMode {

    private DcMotorEx shooter;
    private CRServo sstanga, sdreapta;
    private DcMotor mstanga, mdreapta, keeper;

    // Shooter
    private static final double BASE_POWER = 0.67;
    private static final double BOOST_POWER = 0.70;
    private double shooterPower = BASE_POWER;

    // Feeding
    private static final double SERVO_POWER = 1.0;
    private static final double FEED_TIME = 0.35;
    private static final double PAUSE_TIME = 1.5;
    private static final int BALL_COUNT = 3;

    private boolean shooterOn = false;
    private boolean shootingSequence = false;
    private boolean feeding = false;
    private boolean powerBoosted = false;

    private double lastTime = 0;
    private int ballsShot = 0;

    private boolean rbLast = false;
    private boolean lbLast = false;
    private boolean yLast = false;

    // Keeper limits
    private static final int KEEPER_MAX_TICKS = 280; // 90°
    private int keeperHome = 0;                      // START position

    @Override
    public void runOpMode() {

        shooter   = hardwareMap.get(DcMotorEx.class, "shooter");
        sstanga   = hardwareMap.get(CRServo.class, "sstanga");
        sdreapta  = hardwareMap.get(CRServo.class, "sdreapta");
        mstanga   = hardwareMap.get(DcMotor.class, "mstanga");
        mdreapta  = hardwareMap.get(DcMotor.class, "mdreapta");
        keeper    = hardwareMap.get(DcMotor.class, "keeper");

        initMotors();

        // IMPORTANT: save keeper start position
        keeperHome = keeper.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
            arcadeDrive(
                    -deadzone(gamepad1.left_stick_y),
                    deadzone(gamepad1.left_stick_x)
            );

            // ---------------- KEEPER ----------------
            int pos = keeper.getCurrentPosition();

            // A = move UP to 90°
            if (gamepad1.a && pos < KEEPER_MAX_TICKS) {
                keeper.setPower(-1);
            }
            // B = return to START position
            else if (gamepad1.b) {
                keeper.setPower(1);
            }
            else {
                keeper.setPower(0);
            }

            // ---------------- SHOOTER TOGGLE ----------------
            boolean yNow = gamepad1.y;
            if (yNow && !yLast) shooterOn = !shooterOn;
            yLast = yNow;

            shooter.setPower(shooterOn ? shooterPower : 0);

            // ---------------- SINGLE BALL ----------------
            boolean rbNow = gamepad1.right_bumper;
            if (rbNow && !rbLast && shooterOn) {
                sstanga.setPower(SERVO_POWER);
                sdreapta.setPower(SERVO_POWER);
                sleep((long)(FEED_TIME * 1000));
                sstanga.setPower(0);
                sdreapta.setPower(0);
            }
            rbLast = rbNow;

            // ---------------- 3 BALL AUTO ----------------
            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lbLast && shooterOn && !shootingSequence) {
                shootingSequence = true;
                ballsShot = 0;
                feeding = true;
                powerBoosted = false;
                shooterPower = BASE_POWER;
                lastTime = getRuntime();
            }
            lbLast = lbNow;

            if (shootingSequence) {
                double t = getRuntime();

                if (feeding && t - lastTime >= FEED_TIME) {
                    feeding = false;
                    ballsShot++;
                    lastTime = t;

                    if (ballsShot == 1 && !powerBoosted) {
                        shooterPower = BOOST_POWER;
                        powerBoosted = true;
                    }
                }
                else if (!feeding && ballsShot < BALL_COUNT && t - lastTime >= PAUSE_TIME) {
                    feeding = true;
                    lastTime = t;
                }

                sstanga.setPower(feeding ? SERVO_POWER : 0);
                sdreapta.setPower(feeding ? SERVO_POWER : 0);

                if (ballsShot >= BALL_COUNT) {
                    shootingSequence = false;
                    feeding = false;
                    shooterPower = BASE_POWER;
                }
            }

            telemetry.addData("Keeper Pos", pos);
            telemetry.addData("Keeper Home", keeperHome);
            telemetry.update();
        }
    }

    private void initMotors() {

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mstanga.setDirection(DcMotor.Direction.REVERSE);
        mstanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mdreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        keeper.setDirection(DcMotor.Direction.FORWARD);
        keeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        keeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sdreapta.setDirection(CRServo.Direction.REVERSE);
        sstanga.setPower(0);
        sdreapta.setPower(0);
    }

    private void arcadeDrive(double forward, double rotate) {
        mstanga.setPower(clamp(forward + rotate));
        mdreapta.setPower(clamp(forward - rotate));
    }

    private double clamp(double v) {
        return Math.max(-1, Math.min(1, v));
    }

    private double deadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }
}
