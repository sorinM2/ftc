package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class Hooks {
    private ServoEx servoLeft, servoRight;

    public enum HooksState {
        INIT,
        INTERMEDIATE,
        ACTIVATED,
        READY_LVL_3,
        RELEASE;
    }

    private HooksState state;

    public static double INIT = 0.02;
    public static double INTERMEDIATE = 0.28;
    public static double RELEASE = 0.2;
    public static double ACTIVATED = 0.6     ;
    public static double READY_LVL_3 = 0.6;

    public void Initialize()
    {
        servoLeft = new SimpleServo(hardwareMap, "servoHookLeft", 0, 1);
        servoRight = new SimpleServo(hardwareMap, "servoHookRight", 0, 1);

        setPosition(INIT);
    }

    public void SetState(HooksState state)
    {
        this.state = state;
        switch (state) {
            case INIT:
                setPosition(INIT);

                break;

            case ACTIVATED:
                setPosition(ACTIVATED);

                break;

            case INTERMEDIATE:
                setPosition(INTERMEDIATE);

                break;

            case READY_LVL_3:
                setPosition(READY_LVL_3);

                break;

            case RELEASE:
                setPosition(RELEASE);

                break;
        }
    }
    public static double rightOffset = 0.18d;
    private void setPosition(double position) {
        servoLeft.setPosition(position);
        servoRight.setPosition(position + rightOffset);
    }
}
