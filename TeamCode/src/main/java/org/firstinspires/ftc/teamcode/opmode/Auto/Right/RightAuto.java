package org.firstinspires.ftc.teamcode.opmode.Auto.Right;

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
public class RightAuto extends OpMode {

    // Other Constants
    double initialWaitTime = 1000;
    double shootingStartDelay = 500;
    double shootingDelay = 500;
    double flowerDelay = 2500;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPos = poseFactory.of(53.5, 9, -90);
    private final Pose path1Start = poseFactory.of(53.5, 9, -90);
    private final Pose path1 = poseFactory.of(15, 47.0629, 180);
    private final Pose path1Control1 = poseFactory.of(53.5, 35, 0);
    private final Pose path1Control2 = poseFactory.of(35, 48, 0);
    private final Pose point2 = poseFactory.of(53.2737, 20.851, -142.76);
    private final Pose point2Control1 = poseFactory.of(34.074, 45.8355, 0);
    private final Pose point2Control2 = poseFactory.of(45.5268, 14.5605, 0);
    private final Pose point3 = poseFactory.of(15, 6, -176.4442);
    private final Pose point3Control1 = poseFactory.of(38.5812, 6.6057, 0);
    private final Pose point4 = poseFactory.of(60, 19.5232, -137.932);
    private final Pose point4Control1 = poseFactory.of(39.4303, 4.6922, 0);
    private final Pose point5 = poseFactory.of(48., 124, 94.4625);
    private final Pose point5Control1 = poseFactory.of(62, 133.1585, 0);
    private final Pose point5Control2 = poseFactory.of(48.7034, 103.33, 0);
    private final Pose point6 = poseFactory.of(15, 111.227, 3.7768);
    private final Pose point6Control1 = poseFactory.of(46.7674, 113.7467, 0);

    public Path flower1() {
        return curve(path1Start, path1Control1, path1Control2, path1).linear(path1Start, path1);
    }

    public Path scoreFlower1() {
        return curve(path1, point2Control1, point2Control2, point2).reverseTangent();
    }

    public Path intakeLine() {
        return curve(point2, point3Control1, point3).tangent();
    }

    public Path scoreLine() {
        return curve(point3, point4Control1, point4).reverseTangent();
    }

    public Path flower2() {
        return curve(point4, point5Control1, point5Control2, point5).tangent();
    }

    public Path park() {
        return curve(point5, point6Control1, point6).reverseTangent();
    }

    private Command autoRoutine() {
        return sequential(
                waitMs(initialWaitTime),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        sequential(
                                waitMs(200),
                                intake.closeGate()
                        ),
                        follow(follower, flower1()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerDelay),
                parallel(
                        follow(follower, scoreFlower1()),
                        sequential(
                                waitMs(200),
                                intake.flowerUp(),
                                waitMs(200),
                                intake.intakeDown()//,
                                //intake.intakeOff()
                        )
                ),
                waitMs(shootingStartDelay),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        sequential(
                                waitMs(200),
                                intake.closeGate()
                        ),
                        follow(follower, intakeLine())
                ),
                waitMs(700),
                follow(follower, scoreLine()),
                waitMs(shootingStartDelay),
                intake.transferOn(),
                waitMs(shootingDelay),
                parallel(
                        sequential(
                                waitMs(200),
                                intake.closeGate()
                        ),
                        follow(follower, flower2()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerDelay),
                intake.transferOn(),
                waitMs(shootingDelay),
                follow(follower, park())

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
        shooter.updateTurret(follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));

        double x = follower.pose().x();
        double y = follower.pose().y();

        // Shooter: what the flywheel/hood are actually using
        double targetDist = shooter.getTargetDistance(x, y);
        double targetVel = shooter.getTargetVelocity(targetDist);
        double bottomVel = shooter.getBottomMotorSpeed(); // PIDF feedback motor
        double topVel = shooter.getTopMotorSpeed();

        // Cross-check: distance to the goal the TURRET is aiming at
        double[] turretGoal = shooter.getTurretGoal(y);
        double distToTurretGoal = Math.hypot(turretGoal[0] - (x - 72), turretGoal[1] - (y - 72)) + 6;

        telemetry.addLine("--- Pose ---");
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining in path", follower.remainingDistance());

        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("Target velocity", targetVel);
        telemetry.addData("Bottom velocity (PIDF)", bottomVel);
        telemetry.addData("Top velocity", topVel);
        telemetry.addData("Error (bottom - target)", bottomVel - targetVel);
        telemetry.addData("Top/bottom mismatch", topVel - bottomVel);
        telemetry.addData("Motor power (top/bottom)", "%.2f / %.2f", shooter.getTopMotorPower(), shooter.getBottomMotorPower());

        telemetry.addLine("--- Shot ---");
        telemetry.addData("Shooter target distance", targetDist);
        telemetry.addData("Dist to turret's goal", distToTurretGoal);
        telemetry.addData("Turret goal (x, y)", "%.1f, %.1f", turretGoal[0], turretGoal[1]);
        telemetry.addData("Shot zone", shooter.getShotZone());
        telemetry.addData("Hood position", shooter.getHoodPosition());
        telemetry.addData("Shot table error", shooter.getShotTableError());
        telemetry.addData("Turret servo pos", shooter.getTurretServoPosition());
        telemetry.update();

    }
}
