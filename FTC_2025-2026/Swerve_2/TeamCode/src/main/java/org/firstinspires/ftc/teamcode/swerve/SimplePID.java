package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SimplePID {
    private double kP, kI, kD, kF;
    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean firstUpdate = true;
    private final ElapsedTime timer = new ElapsedTime();

    public SimplePID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();
    }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        firstUpdate = true;
        timer.reset();
    }

    public double update(double error) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0.0) dt = 1e-3;

        integral += error * dt;
        double derivative = 0.0;

        if (!firstUpdate) {
            derivative = (error - lastError) / dt;
        } else {
            firstUpdate = false;
        }

        lastError = error;
        return kP * error + kI * integral + kD * derivative + kF;
    }

    public double updateClamped(double error, double min, double max) {
        return Range.clip(update(error), min, max);
    }
}