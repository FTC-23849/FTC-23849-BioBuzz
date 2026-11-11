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

import java.util.Arrays;

@Config
public class Shooter {
    public HardwareMap hwmap;
    public DcMotorEx topShooter, bottomShooter;
    public ServoImplEx hood, stopper;
    Robot robot;
    public static int numOfPointsRegression = 8;
    public static String[] shotPoints = {
            "20,1060", "25,1065", "30,1050", "40,1135", "50,1205", "60,1285", "70,1300", "80,1340",
            "93,1440", "100,1480", "0,0", "0,0", "0,0", "0,0", "0,0", "0,0"
    };
    public static int maxShotCurveDegree = 4;
    String[] parsedPoints = null;
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
    public double topVelocity = 0;
    public double bottomVelocity = 0;
    int lastBottomPos = 0;
    ElapsedTime bottomTimer = new ElapsedTime();

    void updateBottomVelocity(){
        int pos = bottomShooter.getCurrentPosition();
        double dt = bottomTimer.seconds();
        bottomTimer.reset();
        double estimate = dt > 0 ? (pos - lastBottomPos) / dt : 0;
        lastBottomPos = pos;
        bottomVelocity = inverseOverflow(bottomShooter.getVelocity(), estimate) * bottomEncoderScale;
    }

    static double inverseOverflow(double input, double estimate){
        int real = (int) input & 0xffff;
        real += ((real % 20) / 4) * 0x10000;
        real += Math.round((estimate - real) / (5 * 0x10000)) * 5 * 0x10000;
        return real;
    }

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

    void refreshShotTable(){
        int count = Math.max(0, Math.min(numOfPointsRegression, shotPoints.length));
        String[] used = Arrays.copyOf(shotPoints, count);
        if(Arrays.equals(used, parsedPoints) && maxShotCurveDegree == parsedDegree){
            return;
        }
        parsedPoints = used;
        parsedDegree = maxShotCurveDegree;
        try{
            double[] dists = new double[count];
            double[] vels = new double[count];
            for (int i = 0; i < count; i++) {
                String[] parts = used[i].split(",");
                if(parts.length != 2){
                    throw new IllegalArgumentException("point " + i + " should be distance,velocity but is \"" + used[i] + "\"");
                }
                dists[i] = Double.parseDouble(parts[0].trim());
                vels[i] = Double.parseDouble(parts[1].trim());
            }
            if(numOfPointsRegression > count){
                throw new IllegalArgumentException("numOfPointsRegression is " + numOfPointsRegression + " but list only holds " + count);
            }
            if(count < 2){
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
        double[] goal = robot.turret.calculateGoalPos(robotY + 72); // calculateGoalPos does its own -72
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
        lastBottomPos = bottomShooter.getCurrentPosition();
        bottomTimer.reset();
        topShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initLoop(){

    }

    public void start(){
        hood.setPosition(hoodUp);
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
        topVelocity = -topShooter.getVelocity();
        updateBottomVelocity();
        currVelocity = bottomVelocity;

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
        topVelocity = -topShooter.getVelocity();
        updateBottomVelocity();
        currVelocity = bottomVelocity;

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
