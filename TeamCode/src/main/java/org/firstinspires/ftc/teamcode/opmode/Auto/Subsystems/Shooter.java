package org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Turret;

public class Shooter {

    private final org.firstinspires.ftc.teamcode.opmode.subsystems.Shooter shooter;
    private final Turret turret;

    public Shooter(HardwareMap hwmap, Robot robot) {
        this.shooter = new org.firstinspires.ftc.teamcode.opmode.subsystems.Shooter(hwmap, robot);
        this.turret = new Turret(hwmap, robot);
    }

    public void init() {
        shooter.init();
        turret.init();
    }

    public void updateTurret(double robotX, double robotY, double robotAngleUnsigned) {
        turret.update(robotX, robotY, robotAngleUnsigned);
    }

    public void updateFlywheel(double robotX, double robotY) {
        shooter.update(robotX, robotY);
    }
}
