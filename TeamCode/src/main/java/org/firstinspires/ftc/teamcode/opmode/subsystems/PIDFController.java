package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kV;

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

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = (kP * error) + (kI * integralSum) + (kD * derivative);

        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        }

        return Math.max(-1.0, Math.min(1.0, pid + ff));
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

    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }
}