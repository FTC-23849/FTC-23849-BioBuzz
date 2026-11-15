package org.firstinspires.ftc.teamcode.opmode.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
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

    public AutoRobot(Shooter shooter, Intake intake, Follower follower) {

        this.shooter = shooter;
        this.intake = intake;
        this.follower = follower;

    }

    public Command shoot() {

        return sequential(
                waitMs(AutoConstants.SHOOTING_START_DELAY),
                intake.transferOn(),
                waitMs(AutoConstants.SHOOTING_DELAY),
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
                followOrSkip(path),
                intake.intakeDown()
        );

    }

    public Command followAndFlower(Path path) {

        return parallel(
                followOrSkip(path),
                intake.flowerDown()
        );

    }

    // Same as followAndIntake, but drops the intake delay ms after the path starts
    public Command followAndIntake(Path path, double delay) {

        return parallel(
                followOrSkip(path),
                sequential(
                        waitMs(delay),
                        intake.intakeDown()
                )
        );

    }

    // Same as followAndFlower, but drops the flower delay ms after the path starts
    public Command followAndFlower(Path path, double delay) {

        return parallel(
                followOrSkip(path),
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
                    followOrSkip(path),
                    leaveFlower()
            );
        }
        return followOrSkip(path);

    }

    // Like follow(), but gives up and finishes if the path hasn't made progress
    // (completion hasn't gone up) for AutoConstants.STUCK_TIMEOUT ms, so the auto moves on.
    public Command followOrSkip(Path path) {

        double[] bestCompletion = {0};
        long[] lastProgressTime = {0};

        return Command.build()
                .setStart(() -> {
                    follower.follow(path);
                    bestCompletion[0] = follower.completion();
                    lastProgressTime[0] = System.currentTimeMillis();
                })
                .setDone(() -> {
                    if (follower.atParametricEnd()) {
                        return true;
                    }
                    long now = System.currentTimeMillis();
                    double completion = follower.completion();
                    if (completion > bestCompletion[0] + 0.01) {
                        bestCompletion[0] = completion;
                        lastProgressTime[0] = now;
                    }
                    return now - lastProgressTime[0] >= AutoConstants.STUCK_TIMEOUT;
                });

    }

}
