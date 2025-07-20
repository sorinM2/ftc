package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Pto {
    private ServoEx servoLeft, servoRight;
    public enum PTOStates {
        INIT,
        ACTIVATED;
    }
    public PTOStates state = PTOStates.INIT;
    public static double INITLEFT = 0.2;
    public static double INITRIGHT = 0.15;
    public static double ACTIVATEDLEFT = 0.3;
    public static double ACTIVATEDRIGHT = 0.28;

    public void Initialize()
    {
        servoLeft = new SimpleServo(hardwareMap, "servoPTOLeft", 0, 1);
        servoRight = new SimpleServo(hardwareMap, "servoPTORight", 0, 1);

        SetState(PTOStates.INIT);
    }

    public void SetState(PTOStates state)
    {
        this.state = state;
        switch (state) {
            case INIT:
                servoLeft.setPosition(INITLEFT);
                servoRight.setPosition(INITRIGHT);

                break;

            case ACTIVATED:
                servoLeft.setPosition(ACTIVATEDLEFT);
                servoRight.setPosition(ACTIVATEDRIGHT);

                break;
        }
    }
}
