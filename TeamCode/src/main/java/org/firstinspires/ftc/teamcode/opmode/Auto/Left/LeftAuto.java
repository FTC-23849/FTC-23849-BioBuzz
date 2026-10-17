package org.firstinspires.ftc.teamcode.opmode.Auto.Left;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.opmode.Auto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.opmode.Auto.pedro.Constants;
import org.firstinspires.ftc.teamcode.opmode.subsystems.Robot;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.paths.Path;

@Autonomous
public class LeftAuto extends OpMode {

    // Other Constants
    double initialWaitTime = 5000;
    double shootingDelay = 500;
    double flowerIntakeDelay = 1000;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPos = poseFactory.of(64.5, 132, 90);
    private final Pose flower1 = poseFactory.of(48, 124, 90);
    private final Pose flower1Control1 = poseFactory.of(64.5, 120, 0);
    private final Pose flower1Control2 = poseFactory.of(48, 100, 0);
    private final Pose reverseFlower1 = poseFactory.of(58, 115, 179.7409);
    private final Pose reverseFlower1Control1 = poseFactory.of(48, 115, 0);
    private final Pose flower2 = poseFactory.of(15, 48, -178.066);
    private final Pose flower2Control1 = poseFactory.of(25.6679, 115, 0);
    private final Pose flower2Control2 = poseFactory.of(35, 48, 0);
    private final Pose scoreFlower2 = poseFactory.of(58, 115, -178.7932);
    private final Pose scoreFlower2Control1 = poseFactory.of(35, 48, 0);
    private final Pose scoreFlower2Control2 = poseFactory.of(26, 115, 0);

    public Path flower1() {
        return curve(startPos, flower1Control1, flower1Control2, flower1).constant(flower1);
    }

    public Path reverseFlower1() {
        return curve(flower1, reverseFlower1Control1, reverseFlower1).reverseTangent();
    }

    public Path flower2() {
        return curve(reverseFlower1, flower2Control1, flower2Control2, flower2).tangent();
    }

    public Path scoreFlower2() {
        return curve(flower2, scoreFlower2Control1, scoreFlower2Control2, scoreFlower2).reverseTangent();
    }

    private Command autoRoutine() {
        return sequential(
                waitMs(initialWaitTime),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        intake.closeGate(),
                        follow(follower, flower1()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerIntakeDelay),
                parallel(
                        follow(follower, reverseFlower1()),
                        sequential(
                                waitMs(200),
                                intake.flowerUp(),
                                intake.intakeUp(),
                                intake.intakeOff()
                        )
                ),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        intake.closeGate(),
                        follow(follower, flower2()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerIntakeDelay),
                parallel(
                        follow(follower, scoreFlower2()),
                        sequential(
                                waitMs(200),
                                intake.flowerUp(),
                                intake.intakeUp(),
                                intake.intakeOff()
                        )
                ),
                intake.transferOn(),
                waitMs(shootingDelay)
        );
    }

    @Override
    public void init() {

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPos);
        follower.update();

        robot = new Robot(hardwareMap);
        robot.alliance = "red"; // TODO: confirm this is the correct alliance for this auto

        intake = new Intake(hardwareMap);
        intake.init();

        shooter = new Shooter(hardwareMap, robot);
        shooter.init();

    }

    @Override
    public void start() {

        schedule(autoRoutine());

    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        shooter.updateFlywheel(follower.pose().x(), follower.pose().y());
        shooter.updateTurret(follower.pose().x(), follower.pose().y(), follower.pose().heading());

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());

        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining in path", follower.remainingDistance());
        telemetry.update();

    }
}
