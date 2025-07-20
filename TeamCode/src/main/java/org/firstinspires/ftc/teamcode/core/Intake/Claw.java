package org.firstinspires.ftc.teamcode.core.Intake;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Hardware;

@Config
public class Claw {
    ServoEx servoClawRotation, servoClawWrist;
    public enum ClawState
    {
        TRANSFER,
        SCANNING_START,
        SCAN,
        SCAN_AUTO,
        PICK_UP,
        INIT,
        FENCE;
    }
    private final double ROTATION_INIT = 0.78;
    private final double ROTATION_TRANSFER = 0.78;
    private final double ROTATION_SCANNING = 0.21;
    public static double WristInit = 0.6, WristScan = 0.67, WristScanAuto = 0.69, WristPick_Up = 0.75, WristTransfer = 0.26, WristFence = 0.6 ;
    private double RotationPos = 0.0d, WristPos = 0.0d;

    Hardware _hardware;
    public Claw(Hardware hardware)
    {
        _hardware = hardware;
    }

    public void Initialize()
    {
        servoClawRotation = new SimpleServo(hardwareMap, "servoClawRotation", 0, 1);
        servoClawWrist = new SimpleServo(hardwareMap, "servoClawWrist", 0, 1);

        SetState(ClawState.INIT);
    }

    public ClawState state = ClawState.INIT;
    public void SetState(ClawState state)
    {
        this.state = state;
        switch(state )
        {
            case INIT:
                RotationPos = ROTATION_INIT;
                WristPos = WristInit;

                break;

            case PICK_UP:
                WristPos = WristPick_Up;

                break;

            case SCANNING_START:
                WristPos = WristScan;
                RotationPos = ROTATION_SCANNING;

                break;

            case SCAN:
                WristPos = WristScan;

                break;

            case SCAN_AUTO:
                WristPos = WristScanAuto;

                break;

            case TRANSFER:
                RotationPos = ROTATION_TRANSFER;
                WristPos = WristTransfer;
                break;

            case FENCE:
                WristPos = WristFence;
                RotationPos = ROTATION_INIT;

                break;

        }

        servoClawRotation.setPosition(RotationPos);
        servoClawWrist.setPosition(WristPos);
    }

    public void setClawRotation(double rotation) {
        RotationPos = rotation;

        if (RotationPos < ROTATION_SCANNING)
            RotationPos = ROTATION_SCANNING;

        if (RotationPos > ROTATION_INIT)
            RotationPos = ROTATION_INIT;

        servoClawRotation.setPosition(RotationPos);
    }
    public double getClawRotation() {
        return RotationPos;
    }
}
