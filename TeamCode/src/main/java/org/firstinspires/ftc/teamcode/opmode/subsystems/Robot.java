package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    public HardwareMap hwmap;
    public Telemetry telemetry;
    public Drivetrain dt;
    public Intake intake;
    public Misc misc;
    public Shooter shooter;
    public Turret turret;
    public List<LynxModule> hubs;
    ElapsedTime loopTimer = new ElapsedTime();
    double loopTime = 0;

    public String alliance;
    public boolean aim;
    public Robot(HardwareMap hwmap, Telemetry telemetry){
        this.hwmap = hwmap;
        this.telemetry = telemetry;
        dt = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        misc = new Misc(hwmap);
        shooter = new Shooter(hwmap, this);
        turret = new Turret(hwmap, this);

    }

    public Robot(HardwareMap hwmap){
        this.hwmap = hwmap;
        dt = new Drivetrain(hwmap);
        intake = new Intake(hwmap);
        misc = new Misc(hwmap);
        shooter = new Shooter(hwmap, this);
        turret = new Turret(hwmap, this);

    }

    public void init(){
        hubs = hwmap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        dt.init();
        intake.init();
        misc.init();
        shooter.init();
        turret.init();
    }

    public void initLoop(){
        dt.initLoop();
        intake.initLoop();
        misc.initLoop();
        shooter.initLoop();
        turret.initLoop();
    }

    public void start(){
        dt.start();
        intake.start();
        misc.start();
        shooter.start();
        turret.start();
    }

    public void stop(){
        dt.stop();
        intake.stop();
        misc.stop();
        shooter.stop();
        turret.stop();
    }

    public boolean testingShooter = false;
    public boolean testingTurret = false;
    public double hoodPositionTesting = hoodDown;
    public double turretPositionTesting = 0;
    public double flywheelSpeedTesting = 0;
    public double turretAdjustment = 0;
    public double powerAdjustment = 0;

    public void update(boolean aim, boolean sotm, String Alliance){
        loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        alliance = Alliance;
        this.aim = aim;
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        dt.update(sotm);
        intake.update();
        misc.update();
        shooter.update(aim);
        turret.update(aim);
    }

    public void addTelemetry(){
        telemetry.addData("Loop time (ms)", loopTime);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Aim", aim);
        telemetry.addData("Robot x", dt.getRobotPos().getX(DistanceUnit.INCH));
        telemetry.addData("Robot y", dt.getRobotPos().getY(DistanceUnit.INCH));
        telemetry.addData("Robot heading (deg)", dt.getRobotAngle());
        telemetry.addData("Pinpoint encoder X", dt.pinpoint.getEncoderX());
        telemetry.addData("Pinpoint encoder Y", dt.pinpoint.getEncoderY());
        telemetry.addData("Shooter target x", shooter.targetX);
        telemetry.addData("Shooter target y", shooter.targetY);
        telemetry.addData("Shooter distance from target", shooter.targetDist);
        telemetry.addData("Flywheel current velocity", shooter.currVelocity);
        telemetry.addData("Flywheel target velocity", shooter.targetVelocity);
        telemetry.addData("Flywheel error (curr - target)", shooter.velocityError);        telemetry.addData("Turret angle (deg)", turret.turretAngle);
        telemetry.addData("Shot zone", shooter.shotZone + 1);
        telemetry.addData("Shot table error", shooter.shotTableError);
        telemetry.addData("Left turret servo pos", turret.leftTurret.getPosition());
        telemetry.addData("Right turret servo pos", turret.rightTurret.getPosition());
    }
}
