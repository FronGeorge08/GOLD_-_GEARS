package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterHardware {

    public DcMotorEx shooter;
    public DcMotor intake;

    public CRServo servoS;
    public CRServo servoR;

    public Servo shooterCover;

    public ShooterHardware(HardwareMap hw) {

        shooter = hw.get(DcMotorEx.class, "shooter");
        intake = hw.get(DcMotor.class, "intake");

        servoS = hw.get(CRServo.class, "servoS");
        servoR = hw.get(CRServo.class, "servoR");

        shooterCover = hw.get(Servo.class, "servo_shooter_cover");

        // ---------------- SHOOTER ----------------

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PIDF tuning
        // These are GOOD starting values
        // Tune later if needed

        PIDFCoefficients pidf = new PIDFCoefficients(
                46,
                0,
                2,
                13.5
        );

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        // ---------------- INTAKE ----------------

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- SERVOS ----------------

        servoR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterCover.setDirection(Servo.Direction.REVERSE);
    }

    public void stopFeed() {

        servoS.setPower(0);
        servoR.setPower(0);
    }

    public void openCover() {

        shooterCover.setPosition(0);
    }

    public void closeCover() {

        shooterCover.setPosition(0.9);
    }
}