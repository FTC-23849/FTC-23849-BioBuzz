package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Shooter {
    public HardwareMap hwmap;
    public DcMotorEx topShooter, bottomShooter;
    public ServoImplEx hood, stopper;
    Robot robot;
    public static String shotDistances  = "25,40,50,60,70,80,93,100";
    public static String shotVelocities = "1040,1120,1200,1300,1350,1390,1440,1480";
    public static int maxShotCurveDegree = 3;
    String parsedDistances = null;
    String parsedVelocities = null;
    int parsedDegree = -1;
    PolynomialFunction shotCurve = null;
    public String shotTableError = "table not parsed yet";
    public int shotZone = 0;
    public double launchAngle = 0;
    public double hoodAngle = 0;

    PIDFController flywheelPIDF;
    double targetX = 0;
    double targetY = 0;
    double targetDist = 0;
    double currVelocity = 0;
    double targetVelocity = 0;
    double velocityError = 0;
    VoltageSensor voltageSensor;
    ElapsedTime voltageTimer = new ElapsedTime();
    double batteryVoltage = nominalVoltage;

    void updateBatteryVoltage(){
        if(voltageTimer.milliseconds() > 250){
            double reading = voltageSensor.getVoltage();
            if(reading > 0){
                batteryVoltage = 0.8 * batteryVoltage + 0.2 * reading;
            }
            voltageTimer.reset();
        }
    }

    double compensateVoltage(double power){
        return Math.max(-1.0, Math.min(1.0, power * nominalVoltage / batteryVoltage));
    }

    double[] parseList(String text){
        String[] values = text.trim().split(",", -1);
        double[] parsed = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            parsed[i] = Double.parseDouble(values[i].trim());
        }
        return parsed;
    }

    void refreshShotTable(){
        if(shotDistances.equals(parsedDistances) && shotVelocities.equals(parsedVelocities)
                && maxShotCurveDegree == parsedDegree){
            return;
        }
        parsedDistances = shotDistances;
        parsedVelocities = shotVelocities;
        parsedDegree = maxShotCurveDegree;
        try{
            double[] dists = parseList(shotDistances);
            double[] vels = parseList(shotVelocities);
            if(dists.length != vels.length){
                throw new IllegalArgumentException(dists.length + " distances but " + vels.length + " velocities");
            }
            if(dists.length < 2){
                throw new IllegalArgumentException("need at least 2 points");
            }
            WeightedObservedPoints points = new WeightedObservedPoints();
            for (int i = 0; i < dists.length; i++) {
                points.add(dists[i], vels[i]);
            }
            int degree = Math.max(1, Math.min(maxShotCurveDegree, dists.length - 1));
            shotCurve = new PolynomialFunction(PolynomialCurveFitter.create(degree).fit(points.toList()));
            shotTableError = null;
        }catch(Exception e){
            shotCurve = null;
            shotTableError = e.getMessage();
        }
    }

    public double calculateShotSpeed(double distanceFromTarget){
        if(shotTableError != null){
            return 0;
        }
        return Math.max(0, shotCurve.value(distanceFromTarget));
    }

    double entryLaunchAngle(double d){
        if(d <= 0){
            return 90 - hoodMinAngle;
        }
        double h = hiveHeight - shooterHeight;
        double slope = 2 * h / d - Math.tan(Math.toRadians(entryAngle));
        return Math.toDegrees(Math.atan(slope));
    }

    public double calculateLaunchAngle(double distanceFromTarget){
        double d = distanceFromTarget;
        if(d >= hoodLogicDistThreshold || hoodLogicDistThreshold <= closeShotDist){
            return entryLaunchAngle(d);
        }
        double thresholdAngle = entryLaunchAngle(hoodLogicDistThreshold);
        double t = (d - closeShotDist) / (hoodLogicDistThreshold - closeShotDist);
        return closeLaunchAngle + (thresholdAngle - closeLaunchAngle) * t;
    }

    public double calculateHoodPosition(double distanceFromTarget){
        launchAngle = calculateLaunchAngle(distanceFromTarget);
        hoodAngle = Math.max(hoodMinAngle, Math.min(hoodMaxAngle, 90 - launchAngle));
        double t = (hoodAngle - hoodMinAngle) / (hoodMaxAngle - hoodMinAngle);
        return hoodDown + t * (hoodUp - hoodDown);
    }

    public double calculateTargetDist(Pose2D pos){
        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double[] goal = robot.turret.calculateGoalPos(pos);
        targetX = goal[0];
        targetY = goal[1];
        return Math.hypot(targetX - robotX, targetY - robotY);
    }

    public double calculateTargetDist(double robotX, double robotY){
        robotX -= 72;
        robotY -= 72;
        double[] goal = robot.turret.calculateGoalPos(robotY);
        targetX = goal[0];
        targetY = goal[1];
        return Math.hypot(targetX - robotX, targetY - robotY);
    }



    public Shooter(HardwareMap hwmap, Robot robot){
        this.hwmap = hwmap;
        this.robot = robot;
    }

    public void release(){
        stopper.setPosition(releaseBall);
    }

    public void block(){
        stopper.setPosition(blockBall);
    }

    public void init(){
        topShooter = hwmap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hwmap.get(DcMotorEx.class, "bottomShooter");
        hood = hwmap.get(ServoImplEx.class, "hood");
        stopper = hwmap.get(ServoImplEx.class, "stopper");
        flywheelPIDF = new PIDFController(0, kP, kI, kD, kS, kV);
        voltageSensor = hwmap.voltageSensor.iterator().next();
        batteryVoltage = voltageSensor.getVoltage();
        if(batteryVoltage <= 0){
            batteryVoltage = nominalVoltage;
        }
        voltageTimer.reset();
        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setPosition(hoodUp);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public void update(boolean aim){
        //re-read gains every loop so FTC Dashboard edits apply live
        flywheelPIDF.setPID(kP, kI, kD);
        flywheelPIDF.setFeedforward(kS, kV);
        flywheelPIDF.setMaxIntegralPower(maxIntegralPower);

        refreshShotTable();
        updateBatteryVoltage();

        //target and distance are always calculated so they show in telemetry
        targetDist = calculateTargetDist(robot.dt.getRobotPos());
        currVelocity = -topShooter.getVelocity();

        boolean tableOk = shotTableError == null;
        if(robot.testingShooter){
            hood.setPosition(robot.hoodPositionTesting);
        }else if(aim){
            hood.setPosition(calculateHoodPosition(targetDist));
        }else{
            hood.setPosition(hoodUp);
        }

        if(aim && (robot.testingShooter || tableOk)){
            if(robot.testingShooter){
                targetVelocity = robot.flywheelSpeedTesting;
            }else{
                targetVelocity = Math.max(0, calculateShotSpeed(targetDist) + robot.powerAdjustment);
            }
            flywheelPIDF.setTargetVelocity(targetVelocity);
            double PIDOutputPower = compensateVoltage(flywheelPIDF.update(currVelocity));
            topShooter.setPower(PIDOutputPower);
            bottomShooter.setPower(PIDOutputPower);
        }else{
            targetVelocity = 0;
            flywheelPIDF.reset();
            flywheelPIDF.setTargetVelocity(0);
            topShooter.setPower(0);
            bottomShooter.setPower(0);
        }
        velocityError = currVelocity - targetVelocity;

    }

    public void update(double robotX, double robotY){
        //re-read gains every loop so FTC Dashboard edits apply live
        flywheelPIDF.setPID(kP, kI, kD);
        flywheelPIDF.setFeedforward(kS, kV);
        flywheelPIDF.setMaxIntegralPower(maxIntegralPower);

        refreshShotTable();
        updateBatteryVoltage();

        //target and distance are always calculated so they show in telemetry
        targetDist = calculateTargetDist(robotX, robotY);
        currVelocity = -topShooter.getVelocity();

        boolean tableOk = shotTableError == null;
        if(robot.testingShooter){
            hood.setPosition(robot.hoodPositionTesting);
        }else{
            hood.setPosition(calculateHoodPosition(targetDist));
        }

        if((robot.testingShooter || tableOk)){
            if(robot.testingShooter){
                targetVelocity = robot.flywheelSpeedTesting - 60;
            }else{
                targetVelocity = Math.max(0, calculateShotSpeed(targetDist) + robot.powerAdjustment);
            }
            flywheelPIDF.setTargetVelocity(targetVelocity);
            double PIDOutputPower = compensateVoltage(flywheelPIDF.update(currVelocity));
            topShooter.setPower(PIDOutputPower);
            bottomShooter.setPower(PIDOutputPower);
        }else{
            targetVelocity = 0;
            flywheelPIDF.reset();
            flywheelPIDF.setTargetVelocity(0);
            topShooter.setPower(0);
            bottomShooter.setPower(0);
        }
        velocityError = currVelocity - targetVelocity;

    }
}
