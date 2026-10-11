package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

public class Intake {
    public HardwareMap hwmap;
    public DcMotorEx leftIntake, rightIntake;
    public ServoImplEx dropdownIntake, flowerIntake;
    public Intake(HardwareMap hwmap){
        this.hwmap = hwmap;
    }

    public void transferBalls(){
        leftIntake.setPower(transferSpeed);
        rightIntake.setPower(transferSpeed);
    }
    public void reverseIntake(){
        leftIntake.setPower(reverseIntakePower);
        rightIntake.setPower(reverseIntakePower);
    }

    public void stopIntake(){
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
    }

    public enum IntakeState {
        UP,
        DOWN,
        FLOWER
    }

    public void runIntake(IntakeState intakeState){
        switch (intakeState) {
            case UP:
                dropdownIntake.setPosition(dropDownIntakeUp);
                break;
            case DOWN:
                dropdownIntake.setPosition(dropDownIntakeDown);
                break;
            case FLOWER:
                dropdownIntake.setPosition(dropDownIntakeFlower);
                break;
        }
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
    }

    public void flowerIntakeDown(){
        flowerIntake.setPosition(flowerIntakeDown);
    }

    public void flowerIntakeUp(){
        flowerIntake.setPosition(flowerIntakeUp);
    }

    public void dropdownUp(){
        dropdownIntake.setPosition(dropDownIntakeUp);
    }

    public void init(){
        leftIntake = hwmap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hwmap.get(DcMotorEx.class, "rightIntake");
        dropdownIntake = hwmap.get(ServoImplEx.class, "dropdownIntake");
        flowerIntake = hwmap.get(ServoImplEx.class, "flowerIntake");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initLoop(){

    }

    public void start(){

    }

    public void stop(){

    }

    public void update(){

    }
}
