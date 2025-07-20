package org.firstinspires.ftc.teamcode.core.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.core.Differential;
import org.firstinspires.ftc.teamcode.core.Hardware;

@Config
public class DifferentialClaw extends Differential {
    public DifferentialClaw(Hardware hardware, String LeftServo, String RightServo, double offsetRight) {
        super(hardware, LeftServo, RightServo, offsetRight);
    }
    public static double LINEAR_ANGLE_INIT = 0.38;
    public static double Rotation_ANGLE_INIT = 0.31;
    public static double LINEAR_ANGLE_SCAN = 0.38;
    public static double Rotation_ANGLE_SCAN = 0.31;
    public static double LINEAR_ANGLE_PICKUP = 0.38;
    public static double Rotation_ANGLE_PICKUP = 0.31;
    public static double LINEAR_ANGLE_TRANSFER = 0.04;
    public static double Rotation_ANGLE_TRANSFER = 0.135;

    public void Initialize()
    {
        SetState(ClawDifferentialStates.INIT);
    }

    public enum ClawDifferentialStates
    {
        INIT,
        PICKUP,
        TRANSFER,
        SCAN
    }
    public void SetState(ClawDifferentialStates state)
    {
        switch (state)
        {
            case INIT:
                setPosition(LINEAR_ANGLE_INIT, Rotation_ANGLE_INIT);
                break;
            case PICKUP:
                setPosition(LINEAR_ANGLE_PICKUP, Rotation_ANGLE_PICKUP);
                break;
            case TRANSFER:
                setPosition(LINEAR_ANGLE_TRANSFER, Rotation_ANGLE_TRANSFER);
                break;
            case SCAN:
                setPosition(LINEAR_ANGLE_SCAN, Rotation_ANGLE_SCAN);
                break;
        }
    }
}
