package org.firstinspires.ftc.teamcode.core.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.core.Differential;
import org.firstinspires.ftc.teamcode.core.Hardware;

@Config
public class DifferentialIntake extends Differential {
    public DifferentialIntake(Hardware hardware, String LeftServo, String RightServo, double offsetRight) {
        super(hardware, LeftServo, RightServo, offsetRight);
    }

    public static double LINEAR_ANGLE_INIT = 0.3;
    public static double Rotation_ANGLE_INIT = -0.2;
    public static double LINEAR_ANGLE_SCAN = 0.43;
    public static double Rotation_ANGLE_SCAN = 0.08;
    public static double LINEAR_ANGLE_PICKUP = 0.5;
    public static double Rotation_ANGLE_PICKUP = 0.08;
    public static double LINEAR_ANGLE_TRANSFER = 0.2;
    public static double Rotation_ANGLE_TRANSFER = 0.1;

    public void Initialize()
    {
        SetState(IntakeDifferentialStates.INIT);
    }

    public enum IntakeDifferentialStates
    {
        INIT,
        PICKUP,
        SCAN,
        TRANSFER
    }
    public void SetState(DifferentialIntake.IntakeDifferentialStates state)
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
