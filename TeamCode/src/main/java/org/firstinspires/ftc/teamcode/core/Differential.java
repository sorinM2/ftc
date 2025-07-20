package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Differential {
    protected ServoEx servoLeft, servoRight;
    private double linearAngle = 0.f, rotationAngle = 0.f;

    protected double offsetRight = 0.3;

    private Hardware _hardware;

    public Differential(Hardware hardware, String LeftServo, String RightServo, double offsetRight)
    {
        servoLeft = new SimpleServo(hardwareMap, LeftServo, 0, 1);
        servoRight = new SimpleServo(hardwareMap, RightServo, 0, 1);
        this.offsetRight = offsetRight;
    }
    public void setPosition(double linearAngle, double rotationAngle) {
        this.linearAngle = linearAngle;
        this.rotationAngle = rotationAngle;

        double posleft = linearAngle + rotationAngle;
        servoLeft.setPosition(posleft);
        double posright = linearAngle - rotationAngle + offsetRight;
        servoRight.setPosition(posright);
    }

    public double getLinearAngle()
    {
        return linearAngle;
    }

    public double getRotationAngle()
    {
        return rotationAngle;
    }
}
