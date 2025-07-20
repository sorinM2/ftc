package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakeDifferential extends Differential{
    public OuttakeDifferential(Hardware hardware, String LeftServo, String RightServo, double offsetRight) {
        super(hardware, LeftServo, RightServo, offsetRight);
    }

    public static double LINEAR_ANGLE_INIT = 0.045;
    public static double LINEAR_ANGLE_START = 0.1;
    private final double LINEAR_ANGLE_INTERMEDIATE = 0.3;
    public static double LINEAR_ANGLE_BASKET = 0.36;
    public static double LINEAR_ANGLE_BASKET_AUTO = 0.46;
    private final double LINEAR_ANGLE_BASKET_INTERMEDIATE = 0.3;
    private final double LINEAR_ANGLE_BASKET_ROTATED = 0.38;
    public static double LINEAR_ANGLE_FENCE = 0.055;
    public static double LINEAR_ANGLE_SPECIMEN_READY_TO_RELEASE = 0.7;
    public static double LINEAR_ANGLE_SPECIMEN_RELEASE = 0.7;
    private final double LINEAR_ANGLE_PARKING_AUTO = 0.34;
    private final double LINEAR_ANGLE_HUMAN_PLAYER = 0.4;

    public static double ROTATION_ANGLE_INIT = -0.06;
    private final double ROTATION_ANGLE_INTERMEDIATE = -0.05;
    public static double ROTATION_ANGLE_FORWARD = 0.23;
    public static final double ROTATION_ANGLE_REVERSE = -0.36;
    private final double ROTATION_ANGLE_PARKING_AUTO = -0.05;
    public static double ROTATION_ANGLE_SPECIMEN_RELEASE = 0.24;
    public enum DifferentialStates {
        INIT,
        START,
        INTERMEDIATE,
        BASKET,
        BASKET_INTERMEDIATE,
        BASKET_AUTO,
        FENCE,
        SPECIMEN_READY_TO_RELEASE,
        SPECIMEN_RELEASE,
        HUMAN_PLAYER,
        PARKING_AUTO;
    }

    double linearAngle, rotationAngle;
    public void SetState(DifferentialStates state)
    {
        switch (state) {
            case INIT:
                linearAngle = LINEAR_ANGLE_INIT;
                rotationAngle = ROTATION_ANGLE_INIT;
                break;

            case INTERMEDIATE:
                linearAngle = LINEAR_ANGLE_INTERMEDIATE;
                rotationAngle = ROTATION_ANGLE_FORWARD;
                break;

            case BASKET:
                linearAngle = LINEAR_ANGLE_BASKET;
                rotationAngle = ROTATION_ANGLE_FORWARD;

                break;

            case BASKET_AUTO:
                linearAngle = LINEAR_ANGLE_BASKET_AUTO;
                rotationAngle = ROTATION_ANGLE_FORWARD;

                break;

            case BASKET_INTERMEDIATE:
                linearAngle = LINEAR_ANGLE_BASKET_INTERMEDIATE;
                rotationAngle = ROTATION_ANGLE_FORWARD;

                break;

            case FENCE:
                linearAngle = LINEAR_ANGLE_FENCE;
                rotationAngle = ROTATION_ANGLE_FORWARD;

                break;

            case SPECIMEN_READY_TO_RELEASE:
                linearAngle = LINEAR_ANGLE_SPECIMEN_READY_TO_RELEASE;
                rotationAngle = ROTATION_ANGLE_SPECIMEN_RELEASE;

                break;

            case SPECIMEN_RELEASE:
                linearAngle = LINEAR_ANGLE_SPECIMEN_RELEASE;
                rotationAngle = ROTATION_ANGLE_SPECIMEN_RELEASE;

                break;

            case PARKING_AUTO:
                linearAngle = LINEAR_ANGLE_PARKING_AUTO;
                rotationAngle = ROTATION_ANGLE_PARKING_AUTO;

                break;

            case HUMAN_PLAYER:
                linearAngle = LINEAR_ANGLE_HUMAN_PLAYER;
                rotationAngle = ROTATION_ANGLE_FORWARD;

                break;

            case START:
                linearAngle = LINEAR_ANGLE_START;
                rotationAngle = ROTATION_ANGLE_INIT;

        }
        setPosition(linearAngle, rotationAngle);
    }
}
