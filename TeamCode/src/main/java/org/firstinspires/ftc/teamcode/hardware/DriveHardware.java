package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveHardware {

    public DcMotor LUmotor, LDmotor, RUmotor, RDmotor;

    public DriveHardware(HardwareMap hw) {

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
    }

    public void driveMecanum(double y, double x, double rx) {

        double LU = y + x + rx;
        double LD = y - x + rx;
        double RU = y - x - rx;
        double RD = y + x - rx;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(LU),
                        Math.max(
                                Math.abs(LD),
                                Math.max(Math.abs(RU), Math.abs(RD))
                        )
                )
        );

        LU /= max;
        LD /= max;
        RU /= max;
        RD /= max;

        LUmotor.setPower(LU);
        LDmotor.setPower(LD);
        RUmotor.setPower(RU);
        RDmotor.setPower(RD);
    }
}