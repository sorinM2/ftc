package org.firstinspires.ftc.teamcode.core.Intake;

import static org.firstinspires.ftc.teamcode.Common.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.teamcode.Common.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Common.PIDF;
import org.firstinspires.ftc.teamcode.core.Hardware;
import org.firstinspires.ftc.teamcode.core.Mechanism;

@Config
public class Extendo implements Mechanism {

    private Hardware _hardware;

    public DcMotorEx motor;

    public static int INIT = -10;
    public static  int EXTENDED = 245;
    public static int INTERMEDIATE = 180;
    public static int SCORING = 90;
    public static  int TRANSFER = 0;

    private PIDF pidf;
    public static double P = 0.013, I = 0, D = 0.0017, F = 0.05;
    private int targetPosition, currentPosition, error, offset;

    public static double delta = 0.f;
    private double power, extraPower = 0;
    public static double tolerance = 20;

    public Extendo(Hardware hardware)
    {
        motor = hardwareMap.get(DcMotorEx.class, "motorExtendo");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDF();
        pidf.setCoefficients(P, I, D, F);
        pidf.setTargetSpeed(1);

        targetPosition = INIT;

        state = ExtendoStates.INIT;
    }
    public enum ExtendoStates {
        INIT,
        TRANSFER,
        INTERMEDIATE,
        EXTENDED,
        DETECTION,
        SCORING;
    }

    public ExtendoStates state = ExtendoStates.INIT;
    public void SetState(ExtendoStates state)
    {
        this.state = state;
        switch (state) {
            case INIT:
                targetPosition = INIT;
                pidf.resetReference();

                break;

            case EXTENDED:
                targetPosition = EXTENDED;
                pidf.resetReference();

                break;

            case TRANSFER:
                targetPosition = TRANSFER;
                pidf.resetReference();

                break;

            case INTERMEDIATE:
                targetPosition = INTERMEDIATE;
                pidf.resetReference();

                break;

            case SCORING:
                targetPosition = SCORING;
                pidf.resetReference();

                break;
        }
    }
    @Override
    public void initialize() {

    }

    @Override
    public void update() {
        if ( !usePID )
            return;

        currentPosition = motor.getCurrentPosition() - offset;
        error = targetPosition - currentPosition;

        delta = error;

        if (state == ExtendoStates.TRANSFER )
            pidf.setCoefficients(P * 2, I, D, F);
        else
            pidf.setCoefficients(P, I, D, F);

        power = pidf.getOutput(error);

        if (state == ExtendoStates.TRANSFER && currentPosition > -10)
            power = Math.min(power, -0.99);
        else if ( state == ExtendoStates.TRANSFER)
            power = Math.min(power, -0.2);
        if (power > 1) power = 1;
        if (power < -1) power = -1;



        motor.setPower(power);

        telemetry.addData("Pozitie Extendo", currentPosition);
        telemetry.addData("Putere Extendo", power);
    }

    @Override
    public void finalize() {

    }

    public void setPosition(int position) {
        targetPosition = position;
        pidf.resetReference();
    }
    public boolean usePID = true;
    public void disablePID() {
        usePID = false;
    }
    public void enablePID() {
        usePID = true;
    }
    public int getCurrentPosition() {
        return currentPosition;
    }
    public int getTargetPosition() {
        return targetPosition;
    }
}
