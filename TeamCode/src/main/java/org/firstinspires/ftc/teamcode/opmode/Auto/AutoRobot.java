package org.firstinspires.ftc.teamcode.opmode.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.flowerIntakeDown;
import com.pedropathing.paths.Path;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;

public class AutoRobot {

    private final Shooter shooter;
    private final Intake intake;
    private final Follower follower;
    private final double shootingStartDelay;
    private final double shootingDelay;

    public AutoRobot(Shooter shooter, Intake intake, Follower follower, double shootingStartDelay, double shootingDelay) {

        this.shooter = shooter;
        this.intake = intake;
        this.follower = follower;
        this.shootingStartDelay = shootingStartDelay;
        this.shootingDelay = shootingDelay;

    }

    public Command shoot() {

        return sequential(
                waitMs(shootingStartDelay),
                intake.transferOn(),
                waitMs(shootingDelay),
                intake.closeGate()
        );

    }

    public Command leaveFlower() {

        return sequential(
                waitMs(AutoConstants.LEAVE_FLOWER_UP_DELAY),
                intake.flowerUp(),
                waitMs(200),
                intake.intakeDown()
        );

    }

    public Command followAndIntake(Path path) {

        return parallel(
                follow(follower, path),
                intake.intakeDown()
        );

    }

    public Command followAndFlower(Path path) {

        return parallel(
                follow(follower, path),
                intake.flowerDown()
        );

    }

    // Same as followAndIntake, but drops the intake delay ms after the path starts
    public Command followAndIntake(Path path, double delay) {

        return parallel(
                follow(follower, path),
                sequential(
                        waitMs(delay),
                        intake.intakeDown()
                )
        );

    }

    // Same as followAndFlower, but drops the flower delay ms after the path starts
    public Command followAndFlower(Path path, double delay) {

        return parallel(
                follow(follower, path),
                sequential(
                        waitMs(delay),
                        intake.flowerDown()
                )
        );

    }

    // Follow the path; if the flower intake is down, raise it (leaveFlower) on the way
    public Command followLeavingFlower(Path path) {

        if (intake.flowerIntake.getPosition() == flowerIntakeDown) {
            return parallel(
                    follow(follower, path),
                    leaveFlower()
            );
        }
        return follow(follower, path);

    }

}
