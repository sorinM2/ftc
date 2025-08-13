package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawOutake {
    ServoEx servoClaw;
    public static double CLAW_OPEN = 0.37;
    public static double CLAW_TRANSFER = 0.2;
    public static double CLAW_CLOSED_SPECIMEN = 0.66;
    public static double CLAW_CLOSED_SAMPLE = 0.66;
    public static double CLAW_AUTO_FENCE = 0.18;

    public boolean open = true;
    void Initialize()
    {
        servoClaw = new SimpleServo(hardwareMap, "servoDifferentialClaw", 0, 1);
        closeClawSample();
    }
    public void closeClawSpecimen() {
        servoClaw.setPosition(CLAW_CLOSED_SPECIMEN);
        open = false;
    }

    public void closeClawSample() {
        servoClaw.setPosition(CLAW_CLOSED_SAMPLE);
        open = false;
    }
    public void openClaw() {
        servoClaw.setPosition(CLAW_OPEN);
        open = true;
    }
}
