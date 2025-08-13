package org.firstinspires.ftc.teamcode.core.Intake;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.Set;

@Config
public class Intake {
    ServoEx servoIntake;

    public enum IntakeState
    {
        Intake,
        Outake,
        Intermediate,
        OFF;
    }

    public void Initialize()
    {
        servoIntake = new SimpleServo(hardwareMap, "servoIntake", 0, 1);
        SetState(IntakeState.Intake);
    }
    public static double  IntakePos = 0.88, OutakePos = 0.6, IdlePos = 0.6d;

    public void SetState(IntakeState state)
    {
        double position = 0.0d;

        switch( state )
        {
            case Intake:
                position = IntakePos;
                break;
            case Outake:
                position = OutakePos;
                break;
            case OFF:
                position = IntakePos;
                break;
            case Intermediate:
                position = IdlePos;
                break;
        }
        servoIntake.setPosition(position);

    }

    public double GetPosition()
    {
        return servoIntake.getPosition();
    }

    public void SetPosition(double pos)
    {
        servoIntake.setPosition(pos);
    }
}
