package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "DECODE_AUTO_BLUE", group = "Auto")
public class AUTO_DECODE_BLUE extends LinearOpMode {

    // ================= ODOMETRY (2 WHEEL) =================
    private DcMotor odoLeft, odoRight;

    private static final double ODO_TICKS_PER_REV = 8192;
    private static final double ODO_WHEEL_DIAMETER_MM = 35.0;
    private static final double ODO_TRACK_WIDTH_MM = 280.0;

    private double heading = 0;
    private double yPos = 0;

    private int lastLeft = 0;
    private int lastRight = 0;

    // ================= DRIVE =================
    private DcMotor LUmotor, LDmotor, RUmotor, RDmotor;
    private static final double DRIVE_POWER = 0.5;

    // ================= PID TURN =================
    private static final double TURN_kP = 1.4;
    private static final double TURN_kD = 0.18;
    private static final double TURN_MIN_POWER = 0.12;

    // ================= MECHANISMS =================
    private DcMotorEx Shooter;
    private DcMotor InTake;
    private DcMotor HoseMotor;
    private CRServo LeftServo, RightServo;

    private static final double SHOOTER_POWER = 0.8;
    private static final double INTAKE_POWER = 1.0;
    private static final double HOSE_POWER = 1.0;
    private static final double SERVO_POWER = 1.0;

    private static final double FEED_TIME = 0.35;
    private static final double PAUSE_TIME = 0.7;

    private boolean feeding = false;
    private int ballsShot = 0;
    private double lastTime = 0;

    @Override
    public void runOpMode() {

        // ---- Hardware Map ----
        odoLeft  = hardwareMap.get(DcMotor.class, "odoLeft");
        odoRight = hardwareMap.get(DcMotor.class, "odoRight");

        LUmotor = hardwareMap.get(DcMotor.class, "LUmotor");
        LDmotor = hardwareMap.get(DcMotor.class, "LDmotor");
        RUmotor = hardwareMap.get(DcMotor.class, "RUmotor");
        RDmotor = hardwareMap.get(DcMotor.class, "RDmotor");

        Shooter   = hardwareMap.get(DcMotorEx.class, "Shooter");
        InTake    = hardwareMap.get(DcMotor.class, "InTake");
        HoseMotor = hardwareMap.get(DcMotor.class, "HoseMotor");

        LeftServo  = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        initHardware();

        waitForStart();
        resetOdometry();

        // ================= AUTO SEQUENCE =================

        moveForwardMM(850);
        Shooter.setPower(SHOOTER_POWER);
        sleep(1200);
        shoot(3);

        moveForwardMM(700);
        rotateRadiansPID(Math.PI);

        InTake.setPower(INTAKE_POWER);
        moveForwardMM(600);
        InTake.setPower(0);

        rotateRadiansPID(Math.PI);
        moveForwardMM(600);
        shoot(3);

        stopAll();
    }

    // ================= SHOOT =================
    private void shoot(int balls) {
        ballsShot = 0;
        feeding = true;
        lastTime = getRuntime();

        while (opModeIsActive() && ballsShot < balls) {
            double t = getRuntime();

            if (feeding && t - lastTime >= FEED_TIME) {
                feeding = false;
                ballsShot++;
                lastTime = t;
            } else if (!feeding && t - lastTime >= PAUSE_TIME) {
                feeding = true;
                lastTime = t;
            }

            if (feeding) {
                HoseMotor.setPower(HOSE_POWER);
                LeftServo.setPower(SERVO_POWER);
                RightServo.setPower(-SERVO_POWER);
            } else {
                HoseMotor.setPower(0);
                LeftServo.setPower(0);
                RightServo.setPower(0);
            }
        }

        HoseMotor.setPower(0);
        LeftServo.setPower(0);
        RightServo.setPower(0);
    }

    // ================= DRIVE =================
    private void moveForwardMM(double mm) {
        double startY = yPos;

        while (opModeIsActive() && Math.abs(yPos - startY) < mm) {
            updateOdometry();

            double correction = -heading * 0.8;
            double left = DRIVE_POWER + correction;
            double right = DRIVE_POWER - correction;

            LUmotor.setPower(left);
            LDmotor.setPower(left);
            RUmotor.setPower(right);
            RDmotor.setPower(right);
        }
        stopDrive();
    }

    // ================= PID TURN =================
    private void rotateRadiansPID(double radians) {

        double target = heading + radians;
        double lastError = 0;

        while (opModeIsActive()) {
            updateOdometry();

            double error = target - heading;

            if (Math.abs(error) < Math.toRadians(1.0))
                break;

            double derivative = error - lastError;
            double power = (TURN_kP * error) + (TURN_kD * derivative);

            power = Math.max(-DRIVE_POWER, Math.min(DRIVE_POWER, power));

            if (Math.abs(power) < TURN_MIN_POWER)
                power = Math.copySign(TURN_MIN_POWER, power);

            LUmotor.setPower(power);
            LDmotor.setPower(power);
            RUmotor.setPower(-power);
            RDmotor.setPower(-power);

            lastError = error;
        }
        stopDrive();
    }

    // ================= ODOMETRY =================
    private void resetOdometry() {
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeft = 0;
        lastRight = 0;
        heading = 0;
        yPos = 0;
    }

    private void updateOdometry() {
        int left = odoLeft.getCurrentPosition();
        int right = odoRight.getCurrentPosition();

        int dL = left - lastLeft;
        int dR = right - lastRight;

        lastLeft = left;
        lastRight = right;

        double mmPerTick = Math.PI * ODO_WHEEL_DIAMETER_MM / ODO_TICKS_PER_REV;

        double dLmm = dL * mmPerTick;
        double dRmm = dR * mmPerTick;

        double dTheta = (dRmm - dLmm) / ODO_TRACK_WIDTH_MM;
        heading += dTheta;

        double dY = (dLmm + dRmm) / 2.0;
        yPos += dY;
    }

    // ================= UTIL =================
    private void stopDrive() {
        LUmotor.setPower(0);
        LDmotor.setPower(0);
        RUmotor.setPower(0);
        RDmotor.setPower(0);
    }

    private void initHardware() {
        LUmotor.setDirection(DcMotor.Direction.REVERSE);
        LDmotor.setDirection(DcMotor.Direction.REVERSE);
        RUmotor.setDirection(DcMotor.Direction.FORWARD);
        RDmotor.setDirection(DcMotor.Direction.FORWARD);

        LUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RUmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopAll() {
        stopDrive();
        Shooter.setPower(0);
        InTake.setPower(0);
        HoseMotor.setPower(0);
        LeftServo.setPower(0);
        RightServo.setPower(0);
    }
}
