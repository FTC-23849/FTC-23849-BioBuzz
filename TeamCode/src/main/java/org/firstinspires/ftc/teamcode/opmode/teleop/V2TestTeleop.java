package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "V2TestTeleop", group = "Test")
public class V2TestTeleop extends LinearOpMode {

    public static double DEFAULT_SHOOTER_POWER = 0.5;
    public static double SHOOTER_STEP = 0.05;
    public static double SHOOTER_MAX = 1.0;

    public static double TURRET_CENTER = 0.5;
    public static double TURRET_STEP = 0.05;

    public static double HOOD_MIN = 0.01;
    public static double HOOD_MAX = 0.2;
    public static double HOOD_STEP = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain (gm0 robot-centric mecanum)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake (one motor reversed)
        DcMotor leftIntake = hardwareMap.dcMotor.get("leftIntake");
        DcMotor rightIntake = hardwareMap.dcMotor.get("rightIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter
        DcMotor pollenShooter = hardwareMap.dcMotor.get("pollenShooter");
        pollenShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Turret servos (move together)
        ServoImplEx pollenTurretFront = hardwareMap.get(ServoImplEx.class, "pollenTurretFront");
        ServoImplEx pollenTurretBack = hardwareMap.get(ServoImplEx.class, "pollenTurretBack");
        // Full 500-2500us range so the Axons use their full travel
        pollenTurretFront.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pollenTurretBack.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Hood servo
        Servo pollenHood = hardwareMap.servo.get("pollenHood");

        double turretPos = TURRET_CENTER;
        double hoodPos = HOOD_MIN;
        pollenTurretFront.setPosition(turretPos);
        pollenTurretBack.setPosition(turretPos);
        pollenHood.setPosition(hoodPos);

        boolean prevBack = false, prevStart = false, prevX = false, prevY = false;

        boolean shooterOn = false;
        double shooterPower = DEFAULT_SHOOTER_POWER;
        boolean prevRightBumper = false;
        boolean prevDpadUp = false, prevDpadDown = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ---- Drivetrain ----
            double y = -gamepad1.left_stick_y; // Y stick is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ---- Intake ----
            double intakePower;
            if (gamepad1.right_trigger > 0.1) {
                intakePower = gamepad1.right_trigger;
            } else if (gamepad1.a) {
                intakePower = -0.5; // Hold A to reverse (outtake) at half power
            } else {
                intakePower = 0;
            }
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);

            // ---- Shooter toggle (right bumper, rising edge) ----
            boolean rightBumper = gamepad1.right_bumper;
            if (rightBumper && !prevRightBumper) {
                shooterOn = !shooterOn;
            }
            prevRightBumper = rightBumper;
            // Shooter power adjust (dpad up/down, 0 to SHOOTER_MAX by SHOOTER_STEP)
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadUp && !prevDpadUp) {
                shooterPower += SHOOTER_STEP;
            }
            if (dpadDown && !prevDpadDown) {
                shooterPower -= SHOOTER_STEP;
            }
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;
            shooterPower = Math.round(Math.max(0.0, Math.min(SHOOTER_MAX, shooterPower)) * 100) / 100.0;
            pollenShooter.setPower(shooterOn ? shooterPower : 0);

            // ---- Turret (back = left, start = right, B = center) ----
            boolean back = gamepad1.back;
            boolean start = gamepad1.start;
            if (back && !prevBack) {
                turretPos -= TURRET_STEP;
            }
            if (start && !prevStart) {
                turretPos += TURRET_STEP;
            }
            if (gamepad1.b) {
                turretPos = TURRET_CENTER;
            }
            prevBack = back;
            prevStart = start;
            turretPos = Math.max(0.0, Math.min(1.0, turretPos));
            pollenTurretFront.setPosition(turretPos);
            pollenTurretBack.setPosition(turretPos);

            // ---- Hood (X = down, Y = up, clamped to HOOD_MIN..HOOD_MAX) ----
            boolean xBtn = gamepad1.x;
            boolean yBtn = gamepad1.y;
            if (xBtn && !prevX) {
                hoodPos -= HOOD_STEP;
            }
            if (yBtn && !prevY) {
                hoodPos += HOOD_STEP;
            }
            prevX = xBtn;
            prevY = yBtn;
            hoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos));
            pollenHood.setPosition(hoodPos);

            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Shooter Power", "%.0f%%", shooterPower * 100);
            telemetry.addData("Turret Pos", "%.2f", turretPos);
            telemetry.addData("Hood Pos", "%.2f", hoodPos);
            telemetry.update();
        }
    }
}
