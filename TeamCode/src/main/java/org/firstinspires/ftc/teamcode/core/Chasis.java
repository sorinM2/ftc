package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.robotH;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Commands.Auto.MovementCommand;
import org.firstinspires.ftc.teamcode.Common.StaticVariables;
import org.firstinspires.ftc.teamcode.core.Intake.Extendo;

public class Chasis  {

    private DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    public IMU imu;

    private Hardware _hardware;

    public Chasis(Hardware hardware)
    {
        _hardware = hardware;
    }
    public void initialize() {
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    private double vx, vy, w;
    private double fr, fl, bl, br;
    private double power, theta, sin, cos;
    private double maxx;
    public void updateFieldCentric() {
        YawPitchRollAngles heading = imu.getRobotYawPitchRollAngles();
        robotH = heading.getYaw(AngleUnit.DEGREES) + 90;
        //robotH = MovementCommand.robotH;

        power = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
        theta = Math.atan2(vy, vx) - Math.PI / 4 - (Math.toRadians(robotH) - Math.toRadians(90));

        sin = Math.sin(theta); cos = Math.cos(theta);
        maxx = Math.max(Math.abs(sin), Math.abs(cos));

        if ( vx == 0 && vy == 0)
            power = 0.d;

        fr = power * sin / maxx - w;
        fl = power * cos / maxx + w;
        bl = power * sin / maxx + w;
        br = power * cos / maxx - w;

        maxx = power + Math.abs(w);

        if (maxx > 1) {
            fr /= maxx; fl /= maxx;
            bl /= maxx; br /= maxx;
        }

        motorFrontRight.setPower(fr);
        motorFrontLeft.setPower(fl);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);
    }

    public void updateHang(double power) {
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }

    public void finalize() {

    }

    public void setMovement(double vx, double vy, double w) {
        this.vx = vx;
        this.vy = vy;
        this.w = w;

        if (StaticVariables.teleOp && _hardware._extendo.state == Extendo.ExtendoStates.EXTENDED )
        {
            this.vx *= 0.8;
            this.w *= 0.45;
        }
    }
}
