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
    public static double intakePower = 1.000;
    public static double reverseIntakePower = -1.000;
    public static double dropDownIntakeDown = 0.83;
    public static double dropDownIntakeFlower = 0.7;
    public static double dropDownIntakeUp = 0.63;
    public static double flowerIntakeDown = 0.630;
    public static double flowerIntakeUp = 0.93;
    // Misc

    // Shooter
    public static double blockBall = 0.510;
    public static double releaseBall = 0.380;

    public static double kP = 0.0015;
    public static double kI = 0.0;
    public static double kD = 0.00005;
    public static double kS = 0.1;
    public static double kV = 0.00035;

    public static double hoodUp = 0.25;
    public static double hoodDown = 0.43;


    // Turret
    public static double turretBackCenterPos = 0.58;
    public static double turretOffset = 0;
    public static double turretAdjustmentStep = 0.01;
    public static double powerAdjustmentStep = 10;
    public static double turretRangeDeg = 350;
    public static double rightLimit = 0.219;
    public static double leftLimit = 0.894;
}
