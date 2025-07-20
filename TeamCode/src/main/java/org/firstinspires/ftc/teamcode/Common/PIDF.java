package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    private double P, I, D, F, alpha = 0.8;

    private double output, integralSum = 0, derivative, last_error, delta_error, current_filter, last_filter, speed, feedforward;
    private boolean reset = true, useIF = true;

    private ElapsedTime timer = new ElapsedTime();

    public double getOutput(double error) {

        if (reset) {
            integralSum = 0;
            last_error = error;
            last_filter = 0;
            timer.reset();
            reset = !reset;
            useIF = true;
        }

        delta_error = error - last_error;
        current_filter = last_filter * alpha + delta_error * (1 - alpha);

        derivative = current_filter / timer.seconds();

        integralSum = integralSum + error * timer.seconds();

        if (error < 0) feedforward = -speed;
        else feedforward = speed;

        if (!useIF)
            output = error * P + derivative * D;
        else
            output = error * P + integralSum * I + derivative * D + feedforward * F;

        last_error = error;
        last_filter = current_filter;
        timer.reset();

        return output;
    }

    public void setCoefficients(double P, double I, double D, double F) {
        this.P = P; this.I = I; this.D = D; this.F = F;
    }
    public void setTargetSpeed(double speed) {
        this.speed = speed;
    }
    public void resetReference() {
        reset = true;
    }
    public void disableIF() {
        useIF = false;
    }
    public void enableIF() {
        useIF = true;
    }
}

