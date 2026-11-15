package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Drivetrain
    public static double turretLag = 0.1;
    public static double hoodLag = 0.3;
    public static double shooterLag = 0.1;
    public static double sotmTimeOfFlight = 0.5;
    public static String[] sotmTofPoints = {"20,0.38", "35,0.69", "50,0.675", "60,0.80", "75,0.76", "89,0.79"};
    public static String[] sotmTofPointsBackup = {"20,0.36", "35,0.59", "50,0.675", "60,0.74", "75,0.76", "89,0.78"};
    public static boolean useBackupTof = false;
    public static int sotmIterations = 3;
    public static double sotmVelAlpha = 0.6;
    public static double sotmVelBeta = 0.023;
    public static double sotmVelScale = 0.7;
    public static double sotmOmegaFilter = 0.3;
    public static double sotmOmegaDeadband = 5.73;
    public static double turretPivotForward = 27.18634 / 25.4;
    public static double sotmVelDeadband = 6;
    public static double kP_Lock = 0.21;
    public static double kP_Lock_Small = 0.12;
    public static double kD_Lock = 0.0007;
    public static double PIDDeadband = 3;
    public static double kPRot_Lock = 0.15;
    public static double kDRot_Lock = 0.0002;
    public static double kPRot_Lock_Small = 0.03;
    public static double RotLockDeadband = 5;

    // Intake
    public static double transferSpeed = 1.0;
    public static double transferSpeedAuto = 1.0;
    public static double intakePower = 1.000;
    public static double reverseIntakePower = -1.000;
    public static double dropDownIntakeDown = 0.65;
    public static double dropDownIntakeFlower = 0.4;
    public static double dropDownIntakeUp = 0.34;
    public static double flowerIntakeDown = 0.71;
    public static double flowerIntakeUp = 0.97;
    // Misc

    // Shooter
    public static double blockBall = 0.76;
    public static double releaseBall = 0.980;

    public static double kP = 0.00315;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.15;
    public static double kV = 0.00030;

    public static double nominalVoltage = 13.2;
    public static double maxIntegralPower = 0.2;
    public static double bottomEncoderScale = 28.0 * 30.0 / 20.0 / 8192.0;

    public static double hoodUp = 0.35;
    public static double hoodDown = 0.57;
    public static double hoodMinAngle = 10;
    public static double hoodMaxAngle = 35;

    public static double shooterHeight = 369 / 25.4;
    public static double hiveHeight = 60;
    public static double entryAngle = -30;
    public static double hoodLogicDistThreshold = 60;
    public static double closeShotDist = 25;
    public static double closeLaunchAngle = 70.1;


    // Turret
    public static double turretBackCenterPos = 0.421;
    public static double turretOffset = 0;
    public static double turretAdjustmentStep = 0.01;
    public static double powerAdjustmentStep = 10;
    public static double turretRangeDeg = 360 * (60/20) * (35.0/99);
    public static double rightLimit = 0;
    public static double leftLimit = 1;
    public static double turretServoDifference = 0.019;
}
