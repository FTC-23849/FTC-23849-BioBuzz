package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;

@Config
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;
    public static double turretPositionTesting = 0.5;
    public static double flywheelSpeedTesting = 900;
    boolean testing = false;
    boolean aim = false;
    boolean sotm = false;
    String alliance = "red";
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
    }

    @Override
    public void init_loop() {
        robot.initLoop();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        if(gamepad2.leftBumperWasReleased()){
            //toggle testing, which switches from dynamic aiming and power to static aiming and power,
            // for testing and if the robot break
            testing = !testing;
        }
        robot.testing = testing;
        robot.turretPositionTesting = turretPositionTesting;
        robot.flywheelSpeedTesting = flywheelSpeedTesting;
        robot.update(aim, sotm, alliance);
        robot.addTelemetry();
        telemetry.update();
        robot.dt.driveRobot(gamepad1);

        if(gamepad1.backWasReleased()){
            //toggle aiming (turret + shooter)
            aim = !aim;
        }

        if(gamepad1.left_trigger > 0.7){
            //shoot, so stop blocking and run transfer
            robot.intake.transferBalls();
        }else if(gamepad1.right_trigger > 0.7){
            //run intake with dropdown intake down
            robot.intake.runIntake(Intake.IntakeState.DOWN);
        } else if(gamepad1.right_bumper){
            //run intake without dropdown, maybe do toggle?
            robot.intake.runIntake(Intake.IntakeState.UP);
        }else if(gamepad1.y){
            //run intake with dropdown intake down and flower intake down
            robot.intake.runIntake(Intake.IntakeState.FLOWER);
        }else if(gamepad1.a){
            robot.intake.reverseIntake();
            //run intake in reverse, without changing position of dropdown??
        }else{
            robot.intake.dropdownUp();
            robot.intake.stopIntake();
            robot.intake.flowerIntakeUp();
            robot.shooter.block();
        }

        if((gamepad1.left_trigger > 0.7) && aim){
            robot.shooter.release();
        }

        if(gamepad1.y){
            robot.intake.flowerIntakeDown();
        }


        if(gamepad1.left_bumper){
            //TODO: sotm
            //technically sotm but justin can keep dreaming
            sotm = true;
        }else{
            sotm = false;
        }
        if(gamepad1.dpad_left){
            alliance = "red";
        }
        if(gamepad1.dpad_right){
            alliance = "blue";
        }
        if(gamepad1.x){
            robot.dt.relocalize(alliance);
        }

        if(gamepad1.startWasReleased()){
            //recalibration
            robot.dt.recalibrate();
        }
        //TODO: decide corrections/ locking mechanisms for gp2
        //TODO: additional buttons like locked button / button remapping
        //tbd controls, button for locked pos? buttons for correction?
        // buttons for locking power or turret? gp2 controls?





    }

    @Override
    public void stop() {
        robot.stop();
    }
}
