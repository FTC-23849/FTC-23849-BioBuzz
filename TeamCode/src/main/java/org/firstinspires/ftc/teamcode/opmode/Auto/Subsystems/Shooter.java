package org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Misc;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Turret;

public class Shooter {

    private final org.firstinspires.ftc.teamcode.opmode.subsystems.Shooter shooter;
    private final Turret turret;
    private final Misc misc;

    public Shooter(HardwareMap hwmap, Robot robot) {
        this.shooter = new org.firstinspires.ftc.teamcode.opmode.subsystems.Shooter(hwmap, robot);
        this.turret = new Turret(hwmap, robot);
        this.misc = new Misc(hwmap);
    }

    public void init() {
        shooter.init();
        turret.init();
        misc.init();
        misc.start();
    }

    public boolean canShoot(String alliance) {

        return misc.canShoot(alliance);

    }

    public double getTopMotorSpeed() {

        return shooter.topShooter.getVelocity();

    }

    public double getBottomMotorSpeed() {

        return shooter.bottomShooter.getVelocity();

    }

    public double getTopMotorPower() {
        return shooter.topShooter.getPower();
    }

    public double getBottomMotorPower() {
        return shooter.bottomShooter.getPower();
    }

    // Same distance the flywheel/hood use (calls the opmode Shooter's own method)
    public double getTargetDistance(double robotX, double robotY) {
        return shooter.calculateTargetDist(robotX, robotY);
    }

    // Same velocity lookup the flywheel uses for a given distance
    public double getTargetVelocity(double targetDistance) {
        return shooter.calculateShotSpeed(targetDistance);
    }

    public int getShotZone() {
        return shooter.shotZone + 1;
    }

    public String getShotTableError() {
        return shooter.shotTableError;
    }

    public double getHoodPosition() {
        return shooter.hood.getPosition();
    }

    public double getTurretServoPosition() {
        return turret.leftTurret.getPosition();
    }

    // Goal the turret is aiming at (raw field coords in)
    public double[] getTurretGoal(double robotY) {
        return turret.calculateGoalPos(robotY);
    }

    public void updateTurret(double robotX, double robotY, double robotAngleUnsigned) {
        turret.update(robotX, robotY, robotAngleUnsigned);
    }

    public void updateFlywheel(double robotX, double robotY) {
        shooter.update(robotX, robotY);
    }
}
