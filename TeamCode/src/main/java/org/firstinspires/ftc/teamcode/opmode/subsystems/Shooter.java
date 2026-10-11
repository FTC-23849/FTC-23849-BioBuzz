package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Shooter {
    public HardwareMap hwmap;
    public DcMotorEx topShooter, bottomShooter;
    public ServoImplEx hood, stopper;
    Robot robot;
    public static double[] shotDistances  = {10,20,30,40,50,60,70,80,90,100};
    public static double[] shotVelocities = {1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800};
    PIDFController flywheelPIDF;
    double targetX = 0;
    double targetY = 0;
    double targetDist = 0;
    double currVelocity = 0;
    double targetVelocity = 0;
    double velocityError = 0;


    public double calculateShotSpeed(double distanceFromTarget){
        WeightedObservedPoints points = new WeightedObservedPoints();
        for (int i = 0; i < shotDistances.length; i++) {
            points.add(shotDistances[i], shotVelocities[i]);
        }
        double[] coeffs = PolynomialCurveFitter.create(4).fit(points.toList());
        PolynomialFunction velocityCurve = new PolynomialFunction(coeffs);
        return velocityCurve.value(distanceFromTarget);
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
        hood.setPosition(hoodDown);
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

        //target and distance are always calculated so they show in telemetry
        targetDist = calculateTargetDist(robot.dt.getRobotPos());
        currVelocity = (topShooter.getVelocity() + bottomShooter.getVelocity())/2;

        if(aim){
            targetVelocity = calculateShotSpeed(targetDist);
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
