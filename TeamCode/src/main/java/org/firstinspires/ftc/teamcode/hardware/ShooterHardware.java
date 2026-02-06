package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterHardware {

    public DcMotorEx shooter;
    public DcMotor intake;
    public CRServo servoS, servoR;
    public Servo shooterCover;

    public ShooterHardware(HardwareMap hw) {

        shooter = hw.get(DcMotorEx.class, "shooter");
        intake  = hw.get(DcMotor.class, "intake");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoS = hw.get(CRServo.class, "servoS");
        servoR = hw.get(CRServo.class, "servoR");
        servoR.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterCover = hw.get(Servo.class, "servo_shooter_cover");
        shooterCover.setDirection(Servo.Direction.REVERSE);
    }
}