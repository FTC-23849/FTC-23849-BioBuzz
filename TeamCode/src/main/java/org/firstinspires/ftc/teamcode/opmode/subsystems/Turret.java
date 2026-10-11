package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {
    public HardwareMap hwmap;
    public ServoImplEx leftTurret, rightTurret;
    Pose2D robotPos = null;
    double robotAngleUnsigned;
    boolean aimTurret = false;
    double turretPos = 0;
    double targetX = 0;
    double targetY = 0;
    double turretAngle = 180;
    Robot robot;
    public Turret(HardwareMap hwmap, Robot robot){
        this.hwmap = hwmap;
        this.robot = robot;
    }


    public void init(){
        leftTurret = hwmap.get(ServoImplEx.class, "leftTurret");
        rightTurret = hwmap.get(ServoImplEx.class, "rightTurret");
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public double[] calculateGoalPos(Pose2D pos){
        double robotY = pos.getY(DistanceUnit.INCH);

        //chooses target by picking the cell to aim at, and then
        // picking a point in the hive cell with best angle
        if(robot.alliance.equals("blue")){
            if(robotY < 0){
                //bottom right
                return new double[]{13.1, -19.2};
            }else{
                //top right
                return new double[]{13.1, 19.2};
            }
        }else{
            if(robotY < 0){
                //bottom left
                return new double[]{-13.1, -19.2};
            }else{
                //top left
                return new double[]{-13.1, 19.2};
            }
        }
    }

    public double calculateTurretServoPosition(Pose2D pos, double robotAngleUnsigned){
        double[] targetCoords = calculateGoalPos(pos);

        targetX = targetCoords[0];
        targetY = targetCoords[1];
        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);

        turretAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX));
        if(turretAngle < 0){
            turretAngle+=360;
        }
        turretAngle = ((turretAngle - robotAngleUnsigned) % 360 + 360) % 360;

        double servoPosition = turretBackCenterPos - (turretAngle - 180)/360;

        if (servoPosition < 0){
            servoPosition += 1;
        }
        if (servoPosition > 1){
            servoPosition -= 1;
        }

        return servoPosition;
    }

    public void update(boolean aim){
        aimTurret = aim;
        if(aimTurret){
            robotPos = robot.dt.getRobotPos();
            robotAngleUnsigned = robot.dt.getRobotAngle();
            turretPos = calculateTurretServoPosition(robotPos, robotAngleUnsigned);
            leftTurret.setPosition(turretPos);
            rightTurret.setPosition(turretPos);
        }else{
            //turret is centered, so it points straight backward relative to the robot
            turretAngle = 180;
            leftTurret.setPosition(turretBackCenterPos);
            rightTurret.setPosition(turretBackCenterPos);
        }
    }
}
