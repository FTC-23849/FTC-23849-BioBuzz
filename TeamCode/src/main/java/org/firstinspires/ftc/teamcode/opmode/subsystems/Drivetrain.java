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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Drivetrain {
    public HardwareMap hwmap;
    public Robot robot;
    public DcMotorEx LF, RF, LB, RB;
    public GoBildaPinpointDriver pinpoint;
    public boolean sotm = false;
    public double tof = 0;
    ElapsedTime accelTimer = new ElapsedTime();
    double lastVelX = 0, lastVelY = 0;
    double accelX = 0, accelY = 0;
    double filtVelX = 0, filtVelY = 0;
    double filtOmega = 0;
    public boolean lockedModeEnabled = false;
    private boolean anchorSet = false;
    private double anchorX, anchorY, anchorYaw;
    private double lastHeadingErrorLocked = 0, lastXErrorLocked = 0, lastYErrorLocked = 0;
    private final ElapsedTime lockTimer = new ElapsedTime();


    public Drivetrain(HardwareMap hwmap) {
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

    public void relocalize(String alliance, boolean leftCorner) {
        if (alliance.equals("blue")) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 63.34, leftCorner ? -66.16 : 66.16, AngleUnit.DEGREES, 180));
        } else {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -63.34, leftCorner ? 66.16 : -66.16, AngleUnit.DEGREES, 0));
        }
    }

    public void recalibrate() {
        pinpoint.recalibrateIMU();
    }

    public Pose2D getRobotPos() {
        return getRobotPos(turretLag);
    }

    double getOmega() {
        return Math.abs(filtOmega) < sotmOmegaDeadband ? 0 : filtOmega;
    }

    public Pose2D getRobotPos(double L) {
        Pose2D pos = pinpoint.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);
        if (sotm) {
            double velocityX = filtVelX;
            double velocityY = filtVelY;
            if (Math.hypot(velocityX, velocityY) < sotmVelDeadband) {
                velocityX = 0;
                velocityY = 0;
            }
            double omega = getOmega();
            double releaseHeading = heading + omega * L;
            double rx = turretPivotForward * Math.cos(releaseHeading) - turretPivotLeft * Math.sin(releaseHeading);
            double ry = turretPivotForward * Math.sin(releaseHeading) + turretPivotLeft * Math.cos(releaseHeading);

            double releaseX = pos.getX(DistanceUnit.INCH) + velocityX * L + 0.5 * accelX * L * L + rx;
            double releaseY = pos.getY(DistanceUnit.INCH) + velocityY * L + 0.5 * accelY * L * L + ry;
            double releaseVelX = (velocityX + accelX * L - omega * ry) * sotmVelScale;
            double releaseVelY = (velocityY + accelY * L + omega * rx) * sotmVelScale;

            double predX = releaseX + releaseVelX * sotmTimeOfFlight;
            double predY = releaseY + releaseVelY * sotmTimeOfFlight;
            if (robot != null && robot.alliance != null) {
                for (int i = 0; i < sotmIterations; i++) {
                    double[] goal = robot.turret.calculateGoalPos(new Pose2D(DistanceUnit.INCH, predX, predY, AngleUnit.DEGREES, 0));
                    tof = interpolateTof(Math.hypot(goal[0] - predX, goal[1] - predY));
                    predX = releaseX + releaseVelX * tof;
                    predY = releaseY + releaseVelY * tof;
                }
            } else {
                tof = sotmTimeOfFlight;
            }

            return new Pose2D(DistanceUnit.INCH, predX, predY,
                    AngleUnit.RADIANS, releaseHeading);
        } else {
            double rx = turretPivotForward * Math.cos(heading) - turretPivotLeft * Math.sin(heading);
            double ry = turretPivotForward * Math.sin(heading) + turretPivotLeft * Math.cos(heading);
            return new Pose2D(DistanceUnit.INCH,
                    pos.getX(DistanceUnit.INCH) + rx,
                    pos.getY(DistanceUnit.INCH) + ry,
                    AngleUnit.RADIANS, heading);
        }
    }

    public double getRobotAngle(double L) {
        double angle = getRobotAngle();
        if (sotm) {
            angle += Math.toDegrees(getOmega()) * L;
        }
        return ((angle % 360) + 360) % 360;
    }

    public static double interpolateTof(double dist) {
        String[] points = useBackupTof ? sotmTofPointsBackup : sotmTofPoints;
        int n = points.length;
        double[] d = new double[n];
        double[] t = new double[n];
        for (int i = 0; i < n; i++) {
            String[] parts = points[i].split(",");
            d[i] = Double.parseDouble(parts[0].trim());
            t[i] = Double.parseDouble(parts[1].trim());
        }
        if (n == 1 || dist <= d[0]) return t[0];
        if (dist >= d[n - 1]) return t[n - 1];
        for (int i = 0; i < n - 1; i++) {
            if (dist <= d[i + 1]) {
                double f = (dist - d[i]) / (d[i + 1] - d[i]);
                return t[i] + f * (t[i + 1] - t[i]);
            }
        }
        return t[n - 1];
    }

    public double getRobotAngle() {
        double signedAngle = pinpoint.getHeading(AngleUnit.DEGREES);
        double unsignedAngle = signedAngle;
        if (unsignedAngle < 0) {
            unsignedAngle += 360;
        }
        return unsignedAngle;
    }

    public void init() {
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
        pinpoint.setOffsets(35.7436, 3.4320, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(19.89436789, DistanceUnit.MM);
        pinpoint.recalibrateIMU();
    }

    public void initLoop() {

    }

    public void start() {

    }

    public void stop() {

    }

    public void update(boolean sotm) {
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
        filtVelX += sotmVelFilter * (velX - filtVelX);
        filtVelY += sotmVelFilter * (velY - filtVelY);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        filtOmega += sotmOmegaFilter * (omega - filtOmega);
    }

    public void lockedPos(boolean held) {
        if (held && !lockedModeEnabled) {
            Pose2D pos = pinpoint.getPosition();
            anchorX = -1 * pos.getY(DistanceUnit.METER);
            anchorY = pos.getX(DistanceUnit.METER);
            anchorYaw = ((pos.getHeading(AngleUnit.DEGREES) + 90) % 360 + 360) % 360;
            lastXErrorLocked = 0;
            lastYErrorLocked = 0;
            lastHeadingErrorLocked = 0;
            lockTimer.reset();
            anchorSet = true;
        }
        lockedModeEnabled = held;
        if (!held) {
            anchorSet = false;
        }
    }

    public void runLockedPos() {
        if (!lockedModeEnabled || !anchorSet) {
            return;
        }
        double dt = lockTimer.seconds();
        lockTimer.reset();
        if (dt <= 0) dt = 0.001;

        Pose2D robotPos = pinpoint.getPosition();
        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = ((heading + 90) % 360 + 360) % 360;

        double headingError = anchorYaw - adjustedHeading;
        if (headingError > 180) headingError -= 360;
        else if (headingError < -180) headingError += 360;

        double headingDerivative = (headingError - lastHeadingErrorLocked) / dt;
        double rotPower = kPRot_Lock * Math.signum(headingError) * Math.sqrt(Math.abs(headingError)) + kDRot_Lock * headingDerivative;
        lastHeadingErrorLocked = headingError;

        double xPos = -1 * robotPos.getY(DistanceUnit.METER);
        double yPos = robotPos.getX(DistanceUnit.METER);

        double xError = (anchorX - xPos) * 100;
        double yError = (anchorY - yPos) * 100;
        double dist = Math.sqrt(xError * xError + yError * yError);
        double relAngle = Math.atan2(yError, xError) - Math.toRadians(adjustedHeading);

        xError = dist * -Math.sin(relAngle);
        yError = dist * Math.cos(relAngle);

        double xD = (xError - lastXErrorLocked) / dt;
        double yD = (yError - lastYErrorLocked) / dt;

        double kPLock = (dist < PIDDeadband) ? kP_Lock_Small : kP_Lock;
        double xPower = kPLock * Math.signum(xError) * Math.sqrt(Math.abs(xError)) + kD_Lock * xD;
        double yPower = kPLock * Math.signum(yError) * Math.sqrt(Math.abs(yError)) + kD_Lock * yD;

        lastXErrorLocked = xError;
        lastYErrorLocked = yError;

        LF.setPower(Math.max(-1, Math.min(1, yPower + xPower - rotPower)));
        RF.setPower(Math.max(-1, Math.min(1, yPower - xPower + rotPower)));
        LB.setPower(Math.max(-1, Math.min(1, yPower - xPower - rotPower)));
        RB.setPower(Math.max(-1, Math.min(1, yPower + xPower + rotPower)));
    }
}
