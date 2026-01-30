package org.firstinspires.ftc.teamcode.machanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

public class MecanumDriveSystem {

    private final double TRACKING_WHEEL_DIAMETER = 0;
    private final double TICKS_PER_REV = 0;

    public double metric_kP = 0.5;
    public double angle_kP = 0.7;


    private DcMotor FRMotor, FLMotor, BRMotor, BLMotor, XEncoder, YEncoder;
    private IMU imu;
    public void init(HardwareMap hwMap){
        FRMotor = hwMap.get(DcMotor.class, "RUMotor");
        FLMotor = hwMap.get(DcMotor.class, "LUMotor");
        BRMotor = hwMap.get(DcMotor.class, "RDMotor");
        BLMotor = hwMap.get(DcMotor.class, "LDMotor");

        XEncoder = hwMap.get(DcMotor.class, "insert");
        YEncoder = hwMap.get(DcMotor.class, "insert");
        XEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        YEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(orientation));
    }

    public void drive(double forward, double strafe, double heading){

        if(Math.abs(forward) < 0.05) forward = 0;
        if(Math.abs(strafe) < 0.05) strafe = 0;
        if(Math.abs(heading) < 0.05) heading = 0;

        double FLPower = forward + strafe + heading;
        double FRPower = forward - strafe - heading;
        double BLPower = forward + strafe - heading;
        double BRPower = forward - strafe + heading;

        double maxSpeed = Math.max(Math.abs(FLPower), Math.max(Math.abs(FRPower),
                Math.max(Math.abs(BLPower), Math.abs(BRPower))));

        FLMotor.setPower(FLPower /= maxSpeed);
        FRMotor.setPower(FRPower /= maxSpeed);
        BLMotor.setPower(BLPower /= maxSpeed);
        BRMotor.setPower(BRPower /= maxSpeed);

    }
    /**
     * @param mm_finalXCor coordonata pentru x fata de playing field trebuie sa fie in milimetri
     * @param mm_finalYCor coordonata pentru y fata de playing field trebuie sa fie in milimetri
     * @param finalHeading unghiul la care trebui sa ajunga robotul fata de playing field, TREBUIE
     *                     fie in grade, este convertita si normalizata in radieni automat
     * */
    public void GoToPosition(double mm_finalXCor, double mm_finalYCor, double finalHeading){
        double headingError = 999999;
        double xCorError = 999999;
        double yCorError = 999999;

        do {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double currentXCor = XEncoder.getCurrentPosition() / TICKS_PER_REV * Math.PI * TRACKING_WHEEL_DIAMETER;
            double currentYCor = YEncoder.getCurrentPosition() / TICKS_PER_REV * Math.PI * TRACKING_WHEEL_DIAMETER;

            headingError = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, finalHeading) - currentHeading;
            headingError = AngleUnit.RADIANS.normalize(headingError);
            xCorError = mm_finalXCor - currentXCor;
            yCorError = mm_finalYCor - currentYCor;

            double rawXPower = xCorError * metric_kP;
            double rawYPower = yCorError * metric_kP;

            double frontPower = rawYPower * Math.cos(currentHeading) - rawXPower * Math.sin(currentHeading);
            double strafePower = rawYPower * Math.sin(currentHeading) + rawXPower * Math.cos(currentHeading);

            drive(frontPower, strafePower, headingError * angle_kP);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e){} //trust me
        }while (Math.abs(headingError) > 0.01 || Math.abs(xCorError) > 0.1 || Math.abs(yCorError) > 0.1);
    }
}
