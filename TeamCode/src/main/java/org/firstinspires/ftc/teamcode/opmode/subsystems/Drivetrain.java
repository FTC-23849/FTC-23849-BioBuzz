package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {
    public HardwareMap hwmap;
    public DcMotorEx LF, RF, LB, RB;
    public GoBildaPinpointDriver pinpoint;
    public boolean sotm = false;
    ElapsedTime accelTimer = new ElapsedTime();
    double lastVelX = 0, lastVelY = 0;
    double accelX = 0, accelY = 0;

    public Drivetrain(HardwareMap hwmap){
        this.hwmap = hwmap;
    }

    public void driveRobot(Gamepad gamepad) {

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        LF.setPower((y + x + rx) / denominator);
        LB.setPower((y - x + rx) / denominator);
        RF.setPower((y - x - rx) / denominator);
        RB.setPower((y + x - rx) / denominator);
    }

    public void relocalize(String alliance){
        //TODO: set actual reloc positions
        if(alliance.equals("red")) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -66, -66, AngleUnit.DEGREES, 90));
        } else {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 66, -66, AngleUnit.DEGREES, 90));
        }
    }
    public void recalibrate(){
        pinpoint.recalibrateIMU();
    }

    public Pose2D getRobotPos(){
        Pose2D pos = pinpoint.getPosition();
        if(sotm){
            double velocityX = pinpoint.getVelX(DistanceUnit.INCH);
            double velocityY = pinpoint.getVelY(DistanceUnit.INCH);
            double L = sotmLatency;
            double tof = sotmTimeOfFlight;

            double releaseX = pos.getX(DistanceUnit.INCH) + velocityX * L + 0.5 * accelX * L * L;
            double releaseY = pos.getY(DistanceUnit.INCH) + velocityY * L + 0.5 * accelY * L * L;
            double releaseVelX = velocityX + accelX * L;
            double releaseVelY = velocityY + accelY * L;

            return new Pose2D(DistanceUnit.INCH,
                    releaseX + releaseVelX * tof,
                    releaseY + releaseVelY * tof,
                    AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES));
        }else {
            return pos;
        }
    }
    public double getRobotAngle(){
        double signedAngle = pinpoint.getHeading(AngleUnit.DEGREES);
        double unsignedAngle = signedAngle;
        if (unsignedAngle < 0) {
            unsignedAngle+=360;
        }
        return  unsignedAngle;
    }

    public void init(){
        LF = hwmap.get(DcMotorEx.class, "LF");
        RF = hwmap.get(DcMotorEx.class, "RF");
        LB = hwmap.get(DcMotorEx.class, "LB");
        RB = hwmap.get(DcMotorEx.class, "RB");
        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint = hwmap.get(GoBildaPinpointDriver.class, "pinpoint");
        //TODO: set offsets/ encoder resolution
        pinpoint.setOffsets(0,0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(19.970472542, DistanceUnit.MM);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public void update(boolean sotm){
        this.sotm = sotm;
        pinpoint.update();

        double velX = pinpoint.getVelX(DistanceUnit.INCH);
        double velY = pinpoint.getVelY(DistanceUnit.INCH);
        double dt = accelTimer.seconds();
        accelTimer.reset();
        if (dt > 0) {
            accelX += sotmAccelFilter * ((velX - lastVelX) / dt - accelX);
            accelY += sotmAccelFilter * ((velY - lastVelY) / dt - accelY);
        }
        lastVelX = velX;
        lastVelY = velY;
    }
}
