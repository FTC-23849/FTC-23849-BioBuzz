package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kV;
    private double maxIntegralPower = 0.2;

    private double targetVelocity;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDFController(double targetVelocity, double kP, double kI, double kD, double kS, double kV) {
        this.targetVelocity = targetVelocity;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        timer.reset();
    }

    public double update(double currentVelocity) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        double error = targetVelocity - currentVelocity;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        }

        double withoutI = (kP * error) + (kD * derivative) + ff;
        double output = withoutI + (kI * integralSum);
        boolean pushingPastMax = output >= 1.0 && error > 0;
        boolean pushingPastMin = output <= -1.0 && error < 0;
        if (!pushingPastMax && !pushingPastMin) {
            integralSum += error * dt;
        }

        if (kI != 0) {
            double maxSum = maxIntegralPower / Math.abs(kI);
            integralSum = Math.max(-maxSum, Math.min(maxSum, integralSum));
        } else {
            integralSum = 0.0;
        }

        return Math.max(-1.0, Math.min(1.0, withoutI + (kI * integralSum)));
    }

    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    public double getTargetVelocity() { return targetVelocity; }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setMaxIntegralPower(double maxIntegralPower) {
        this.maxIntegralPower = Math.abs(maxIntegralPower);
    }

    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }
}