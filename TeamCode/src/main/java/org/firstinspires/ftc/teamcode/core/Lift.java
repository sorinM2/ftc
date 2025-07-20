package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.PIDF;

@Config
public class Lift {
    public DcMotor motorUp, motorDown, encoder;
    public enum LiftStates {
        INIT,
        LOW_BASKET,
        AUTO_TRANSFER,
        HIGH_BASKET,
        HIGH_BASKET_AUTO,
        LOW_CHAMBER,
        HIGH_CHAMBER,
        HIGH_CHAMBER_PRELOAD,
        HIGH_CHAMBER_RELEASE,
        LOW_CHAMBER_RELEASE,
        HANG_LVL2,
        HANG_LVL3,
        CAMERA;
    }
    public static int delta = 0;
    private PIDF pidf;
    public static double ascentP = 0.017 , ascentI = 0, ascentD = 0.00005, ascentF = 0;
    public static double descentP = 0.002, descentI = 0, descentD = 0.0001, descentF = 0;
    public static double CT = 0.0005;

    public int targetPosition = 0, lastTargetPosition = 0, currentPosition = 0, error, offset = 0;

    private double power;

    private boolean usePID = true;

    private final int INIT = 0;
    private final int LOW_BASKET = 280;
    private final int AUTO_TRANSFER = 140;
    private final int HIGH_BASKET = 610;
    private final int HIGH_BASKET_AUTO = 675;
    private final int LOW_CHAMBER = 35;
    public static int LOW_CHAMBER_RELEASE = 125;
    public static int HIGH_CHAMBER = 480;
    public static int HIGH_CHAMBER_PRELOAD = 480;
    public static int HIGH_CHAMBER_RELEASE = 600;
    public static int HANG_LVL2 = 350;
    public static int HANG_LVL3 = 695;
    public static int CAMERA = 450;

    public Lift()
    {
        pidf = new PIDF();
        pidf.setTargetSpeed(1);


    }

    public void Initialize()
    {
        motorUp = hardwareMap.get(DcMotor.class, "motorLiftUp");
        motorDown = hardwareMap.get(DcMotor.class, "motorLiftDown");

        motorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoder = motorUp;

        SetState(LiftStates.INIT);
    }

    LiftStates state = LiftStates.INIT;
    public void SetState(LiftStates state)
    {
        this.state = state;
        switch (state) {
            case INIT:
                setPosition(INIT);

                break;

            case LOW_BASKET:
                targetPosition = LOW_BASKET;
                pidf.resetReference();

                break;

            case HIGH_BASKET:
                targetPosition = HIGH_BASKET;
                pidf.resetReference();

                break;

            case HIGH_BASKET_AUTO:
                targetPosition = HIGH_BASKET_AUTO;
                pidf.resetReference();

                break;

            case LOW_CHAMBER:
                targetPosition = LOW_CHAMBER;
                pidf.resetReference();

                break;

            case HIGH_CHAMBER:
                targetPosition = HIGH_CHAMBER;
                pidf.resetReference();

                break;

            case HIGH_CHAMBER_PRELOAD:
                targetPosition = HIGH_CHAMBER_PRELOAD;
                pidf.resetReference();

                break;

            case LOW_CHAMBER_RELEASE:
                targetPosition = LOW_CHAMBER_RELEASE;
                pidf.resetReference();

                break;

            case HIGH_CHAMBER_RELEASE:
                targetPosition = HIGH_CHAMBER_RELEASE;
                pidf.resetReference();

                break;

            case HANG_LVL2:
                targetPosition = HANG_LVL2;
                pidf.resetReference();

                break;

            case HANG_LVL3:
                targetPosition = HANG_LVL3;
                pidf.resetReference();

                break;

            case CAMERA:
                targetPosition = CAMERA;
                pidf.resetReference();

                break;

        }
    }


    public void Update()
    {

        lastTargetPosition = targetPosition;

        currentPosition = encoder.getCurrentPosition() - offset;

        error = targetPosition - currentPosition;

        if (targetPosition > currentPosition) pidf.setCoefficients(ascentP, ascentI, ascentD, ascentF);
        else pidf.setCoefficients(descentP, descentI, descentD, descentF);


        delta = error;


        if (usePID) {
            power = pidf.getOutput(error) + CT * currentPosition;

            if (power > 1) power = 1;
            if (power < -1) power = -1;

            //if (state == LiftStates.INIT && (10 < currentPosition && currentPosition < 40)) power = Math.min(-0.3, power);
            if (state == LiftStates.HANG_LVL3) power = Math.max(0.8, power);
        }

        motorUp.setPower(power);
        motorDown.setPower(power);

        telemetry.addData("Pozitie lift", currentPosition);
    }

    public int getCurrentPosition() {
        return currentPosition;
    }
    public void setPower(double power) {
        this.power = power;
    }

    public void disablePID() {
        usePID = false;
    }
    public void enablePID() {
        usePID = true;
    }

    public void setPosition(int position) {
        if (position < INIT) position = INIT;
        if (position > HANG_LVL3) position = HANG_LVL3;

        targetPosition = position;
        pidf.resetReference();
    }
}
