package org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

import org.firstinspires.ftc.teamcode.opmode.subsystems.Constants;

public class Intake {

    public HardwareMap hwmap;
    public DcMotorEx leftIntake, rightIntake;
    public ServoImplEx dropdownIntake, flowerIntake;
    public ServoImplEx stopper;

    public Intake (HardwareMap hardwareMap) {

        hwmap = hardwareMap;

        leftIntake = hwmap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hwmap.get(DcMotorEx.class, "rightIntake");
        dropdownIntake = hwmap.get(ServoImplEx.class, "dropdownIntake");
        flowerIntake = hwmap.get(ServoImplEx.class, "flowerIntake");
        stopper = hwmap.get(ServoImplEx.class, "stopper");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void init() {

        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);

        dropdownIntake.setPosition(dropDownIntakeUp);
        flowerIntake.setPosition(flowerIntakeUp);

    }

    public Command intakeOn() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(intakePower);
                    rightIntake.setPower(intakePower);
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake);

    }

    public Command intakeOff() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake);

    }

    public Command intakeDown() {

        return Command.build()
                .setStart(() -> {
                    dropdownIntake.setPosition(dropDownIntakeDown);
                })
                .setDone(() -> true)
                .requiring(dropdownIntake);

    }

    public Command intakeUp() {

        return Command.build()
                .setStart(() -> {
                    dropdownIntake.setPosition(dropDownIntakeUp);
                })
                .setDone(() -> true)
                .requiring(dropdownIntake);

    }

    public Command flowerDown() {

        return Command.build()
                .setStart(() -> {
                    dropdownIntake.setPosition(dropDownIntakeFlower);
                    flowerIntake.setPosition(flowerIntakeDown);
                })
                .setDone(() -> true)
                .requiring(dropdownIntake, flowerIntake);

    }

    public Command flowerUp() {

        return Command.build()
                .setStart(() -> {
                    flowerIntake.setPosition(flowerIntakeUp);
                })
                .setDone(() -> true)
                .requiring(dropdownIntake, flowerIntake);

    }

    public Command flowerAndIntakeUp() {

        return Command.build()
                .setStart(() -> {
                    dropdownIntake.setPosition(dropDownIntakeUp);
                    flowerIntake.setPosition(flowerIntakeUp);
                })
                .setDone(() -> true)
                .requiring(dropdownIntake, flowerIntake);

    }

    public Command transferOn() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(transferSpeed);
                    rightIntake.setPower(transferSpeed);
                    stopper.setPosition(releaseBall);
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake, stopper);

    }

    public Command transferOff() {

        return Command.build()
                .setStart(() -> {
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);
                    stopper.setPosition(blockBall);
                })
                .setDone(() -> true)
                .requiring(leftIntake, rightIntake, stopper);

    }

    public Command openGate() {

        return Command.build()
                .setStart(() -> {
                    stopper.setPosition(releaseBall);
                })
                .setDone(() -> true)
                .requiring(stopper);

    }

    public Command closeGate() {

        return Command.build()
                .setStart(() -> {
                    stopper.setPosition(blockBall);
                })
                .setDone(() -> true)
                .requiring(stopper);

    }

}
