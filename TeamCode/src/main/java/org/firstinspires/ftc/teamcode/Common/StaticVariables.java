package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StaticVariables {
    public static double robotX, robotY, robotH;

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static void init(HardwareMap hm, Gamepad gm1, Gamepad gm2, Telemetry tel)
    {
        robotX = 0; robotY = 0; robotH = Math.PI / 2;
        hardwareMap = hm;
        gamepad1 = gm1;
        gamepad2 = gm2;
        telemetry = tel;
    }
}
