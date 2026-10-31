package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Drivetrain
    public static double sotmLatency = 0.1;
    public static double sotmTimeOfFlight = 0.5;
    public static double sotmAccelFilter = 0.3;

    // Intake
    public static double transferSpeed = 1.0;
    public static double transferSpeedAuto = 0.7;
    public static double
            intakePower = 1.000;
    public static double reverseIntakePower = -1.000;
    public static double dropDownIntakeDown = 0.57;
    public static double dropDownIntakeFlower = 0.45;
    public static double dropDownIntakeUp = 0.37;
    public static double flowerIntakeDown = 0.7;
    public static double flowerIntakeUp = 0.97;
    // Misc

    // Shooter
    public static double blockBall = 0.510;
    public static double releaseBall = 0.380;

    public static double kP = 0.006;
    public static double kI = 0.0;
    public static double kD = 0.00004;
    public static double kS = 0.1;
    public static double kV = 0.00035;
    public static double nominalVoltage = 13.2;
    public static double maxIntegralPower = 0.2;

    public static double hoodUp = 0.25;
    public static double hoodDown = 0.43;
    public static double hoodMinAngle = 10;
    public static double hoodMaxAngle = 35;

    public static double shooterHeight = 369 / 25.4;
    public static double hiveHeight = 60;
    public static double entryAngle = -30;
    public static double hoodLogicDistThreshold = 60;
    public static double closeShotDist = 25;
    public static double closeLaunchAngle = 70.1;


    // Turret
    public static double turretBackCenterPos = 0.47;
    public static double turretOffset = 0;
    public static double turretAdjustmentStep = 0.01;
    public static double powerAdjustmentStep = 10;
    public static double turretRangeDeg = 350 * (60/20) * (35.0/99);
    public static double rightLimit = 0;
    public static double leftLimit = 1;
}
