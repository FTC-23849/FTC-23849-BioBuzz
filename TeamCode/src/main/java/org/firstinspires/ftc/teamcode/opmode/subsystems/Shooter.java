package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
    public static String shotDistances  = "25,30|30,40,50|50,60,70,80|80,90,100";
    public static String shotVelocities = "1350,1550|1370,1500,1560|1370,1490,1500,1560|1500,1560,1630";
    public static String hoodHeights    = "0.37|0.3|0.28|0.25";
    String parsedDistances = null;
    String parsedVelocities = null;
    String parsedHoodHeights = null;
    PolynomialFunction[] zoneCurves = null;
    double[] zoneEnds = null;
    double[] zoneHoods = null;
    public String shotTableError = "table not parsed yet";
    public int shotZone = 0;

    PIDFController flywheelPIDF;
    double targetX = 0;
    double targetY = 0;
    double targetDist = 0;
    double currVelocity = 0;
    double targetVelocity = 0;
    double velocityError = 0;


    double[][] parseZones(String text){
        String trimmed = text.trim();
        while(trimmed.startsWith("|")){
            trimmed = trimmed.substring(1).trim();
        }
        while(trimmed.endsWith("|")){
            trimmed = trimmed.substring(0, trimmed.length() - 1).trim();
        }
        String[] groups = trimmed.split("[|]", -1);
        double[][] zones = new double[groups.length][];
        for (int z = 0; z < groups.length; z++) {
            String[] values = groups[z].trim().split(",", -1);
            zones[z] = new double[values.length];
            for (int i = 0; i < values.length; i++) {
                zones[z][i] = Double.parseDouble(values[i].trim());
            }
        }
        return zones;
    }

    void refreshShotTable(){
        if(shotDistances.equals(parsedDistances) && shotVelocities.equals(parsedVelocities)
                && hoodHeights.equals(parsedHoodHeights)){
            return;
        }
        parsedDistances = shotDistances;
        parsedVelocities = shotVelocities;
        parsedHoodHeights = hoodHeights;
        try{
            double[][] dists = parseZones(shotDistances);
            double[][] vels = parseZones(shotVelocities);
            double[][] hoods = parseZones(hoodHeights);
            if(dists.length != vels.length || dists.length != hoods.length){
                throw new IllegalArgumentException("zone counts differ: dist " + dists.length
                        + ", vel " + vels.length + ", hood " + hoods.length);
            }
            PolynomialFunction[] curves = new PolynomialFunction[dists.length];
            double[] ends = new double[dists.length];
            double[] hoodPositions = new double[dists.length];
            for (int z = 0; z < dists.length; z++) {
                if(dists[z].length != vels[z].length){
                    throw new IllegalArgumentException("zone " + (z + 1) + " has "
                            + dists[z].length + " distances but " + vels[z].length + " velocities");
                }
                if(dists[z].length < 2){
                    throw new IllegalArgumentException("zone " + (z + 1) + " needs at least 2 points");
                }
                if(hoods[z].length != 1){
                    throw new IllegalArgumentException("zone " + (z + 1) + " needs exactly 1 hood height");
                }
                WeightedObservedPoints points = new WeightedObservedPoints();
                for (int i = 0; i < dists[z].length; i++) {
                    points.add(dists[z][i], vels[z][i]);
                }
                int degree = Math.min(4, dists[z].length - 1);
                curves[z] = new PolynomialFunction(PolynomialCurveFitter.create(degree).fit(points.toList()));
                ends[z] = dists[z][dists[z].length - 1];
                hoodPositions[z] = hoods[z][0];
            }
            zoneCurves = curves;
            zoneEnds = ends;
            zoneHoods = hoodPositions;
            shotTableError = null;
        }catch(Exception e){
            zoneCurves = null;
            zoneEnds = null;
            zoneHoods = null;
            shotTableError = e.getMessage();
        }
    }

    int findZone(double distanceFromTarget){
        for (int z = 0; z < zoneEnds.length - 1; z++) {
            if(distanceFromTarget <= zoneEnds[z]){
                return z;
            }
        }
        return zoneEnds.length - 1;
    }

    public double calculateShotSpeed(double distanceFromTarget){
        if(shotTableError != null){
            return 0;
        }
        shotZone = findZone(distanceFromTarget);
        return Math.max(0, zoneCurves[shotZone].value(distanceFromTarget));
    }

    public double calculateTargetDist(Pose2D pos){
        //adds distance to midpoint entry to shoot deeper into the goal
        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double[] entry = robot.turret.calculateGoalPos(pos);
        double entryX = entry[0];
        double entryY = entry[1];
        double distance = Math.hypot(entryX - robotX, entryY - robotY);
        //the point 6 in past the entry along the shot line, for telemetry
        targetX = entryX + 6 * (entryX - robotX) / distance;
        targetY = entryY + 6 * (entryY - robotY) / distance;
        return distance + 6;
    }

    public double calculateTargetDist(double robotX, double robotY){
        //adds distance to midpoint entry to shoot deeper into the goal
        robotX -= 72;
        robotY -= 72;
        double[] entry = robot.turret.calculateGoalPos(robotY);
        double entryX = entry[0];
        double entryY = entry[1];
        double distance = Math.hypot(entryX - robotX, entryY - robotY);
        //the point 6 in past the entry along the shot line, for telemetry
        targetX = entryX + 6 * (entryX - robotX) / distance;
        targetY = entryY + 6 * (entryY - robotY) / distance;
        return distance + 6;
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

        refreshShotTable();

        //target and distance are always calculated so they show in telemetry
        targetDist = calculateTargetDist(robot.dt.getRobotPos());
        currVelocity = bottomShooter.getVelocity();

        boolean tableOk = shotTableError == null;
        if(robot.testingShooter){
            hood.setPosition(robot.hoodPositionTesting);
        }else if(aim && tableOk){
            hood.setPosition(zoneHoods[findZone(targetDist)]);
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
            double PIDOutputPower = flywheelPIDF.update(currVelocity);
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

        refreshShotTable();

        //target and distance are always calculated so they show in telemetry
        targetDist = calculateTargetDist(robotX, robotY);
        currVelocity = bottomShooter.getVelocity();

        boolean tableOk = shotTableError == null;
        if(robot.testingShooter){
            hood.setPosition(robot.hoodPositionTesting);
        }else if(tableOk){
            hood.setPosition(zoneHoods[findZone(targetDist)]);
        }else{
            hood.setPosition(hoodUp);
        }

        if((robot.testingShooter || tableOk)){
            if(robot.testingShooter){
                targetVelocity = robot.flywheelSpeedTesting - 60;
            }else{
                targetVelocity = Math.max(0, calculateShotSpeed(targetDist) + robot.powerAdjustment);
            }
            flywheelPIDF.setTargetVelocity(targetVelocity);
            double PIDOutputPower = flywheelPIDF.update(currVelocity);
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
