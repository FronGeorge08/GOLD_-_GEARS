package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class HardWare {

    public DcMotor LUmotor, LDmotor, RUmotor, RDmotor;

    public DcMotor intake;
    public DcMotorEx shooter;
    public CRServo servoS, servoR;
    public Servo servo_shooter_cover;
    public HardWare(HardwareMap hw) {

        LUmotor = hw.get(DcMotor.class, "LUmotor");
        LDmotor = hw.get(DcMotor.class, "LDmotor");
        RUmotor = hw.get(DcMotor.class, "RUmotor");
        RDmotor = hw.get(DcMotor.class, "RDmotor");

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

        shooter = hw.get(DcMotorEx.class, "shooter");
        intake  = hw.get(DcMotor.class, "intake");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo_shooter_cover=hw.get(Servo.class,"servo_shooter_cover");
        servoS = hw.get(CRServo.class, "servoS");
        servoR = hw.get(CRServo.class, "servoR");
        servoR.setDirection(DcMotorSimple.Direction.REVERSE);
        servo_shooter_cover.setDirection(Servo.Direction.REVERSE);
    }
}
