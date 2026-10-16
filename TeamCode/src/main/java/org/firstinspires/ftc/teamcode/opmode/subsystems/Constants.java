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
    public static double intakePower = 1.000;
    public static double reverseIntakePower = -1.000;
    public static double dropDownIntakeDown = 0.6;
    public static double dropDownIntakeFlower = 0.440;
    public static double dropDownIntakeUp = 0.420;
    public static double flowerIntakeDown = 0.480;
    public static double flowerIntakeUp = 0.750;
    // Misc

    // Shooter
    public static double blockBall = 0.190;
    public static double releaseBall = 0.130;
    //seconds the stopper stays open after releasing, so the last ball clears before it closes
    public static double blockDelay = 0.5;

    public static double kP = 0.006;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.03;
    public static double kV = 0.0007;

    public static double hoodUp = 0.25;
    public static double hoodDown = 0.43;


    // Turret
    public static double turretBackCenterPos = 0.5;
    public static double turretRangeDeg = 350;
}
