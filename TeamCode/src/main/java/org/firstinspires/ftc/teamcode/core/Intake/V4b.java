package org.firstinspires.ftc.teamcode.core.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;

import org.firstinspires.ftc.teamcode.Common.StaticVariables;
import org.firstinspires.ftc.teamcode.core.Hardware;

@Config
public class V4b {
    private ServoEx servoV4BLeft , servoV4BRight;

    public enum V4BStates
    {
        INIT,
        TRANSFER,
        SCANNING,
        PICK_UP,
        FENCE;
    }
    public static double InitPos = 0.48, ScanningPos = 0.62, PickupPos = 0.7 , TransferPos = 0.55, FencePos = 0.4;
    private V4BStates state;

    private Hardware _hardware;
    public V4b(Hardware hardware)
    {
        _hardware = hardware;

    }

    public void initialize()
    {
        servoV4BLeft = new SimpleServo(hardwareMap, "servoV4BLeft", 0, 1);
        servoV4BRight = new SimpleServo(hardwareMap, "servoV4BRight", 0, 1);
        SetState(V4BStates.INIT);
    }
    public void SetState(V4BStates state) {
        switch (state) {
            case INIT:
                setV4BPos(InitPos);
                break;
            case PICK_UP:
                setV4BPos(PickupPos);
                break;
            case SCANNING:
                setV4BPos(ScanningPos);
                break;
            case TRANSFER:
                setV4BPos(TransferPos);
                break;
            case FENCE:
                setV4BPos(FencePos);
                break;
        }
    }
    public void setV4BPos(double pos)
    {
        servoV4BLeft.setPosition(pos);
        servoV4BRight.setPosition(pos);
    }
}
