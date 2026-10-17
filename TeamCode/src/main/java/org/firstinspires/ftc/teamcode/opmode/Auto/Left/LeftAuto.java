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
    double initialWaitTime = 10000;
    double shootingStartDelay = 500;
    double shootingDelay = 500;
    double flowerDelay = 3000;

    private Robot robot;
    private Intake intake;
    private Shooter shooter;

    // Pedro
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPos = poseFactory.of(64.5, 132, 90);
    private final Pose flower1 = poseFactory.of(48, 128, 90);
    private final Pose flower1Control1 = poseFactory.of(64.5, 120, 0);
    private final Pose flower1Control2 = poseFactory.of(48, 100, 0);
    private final Pose reverseFlower1 = poseFactory.of(58, 130, 179.7409);
    private final Pose reverseFlower1Control1 = poseFactory.of(48, 115, 0);
    private final Pose flower2 = poseFactory.of(12, 50, -178.066);
    private final Pose flower2Control1 = poseFactory.of(25.6679, 115, 0);
    private final Pose flower2Control2 = poseFactory.of(40, 48, 0);
    private final Pose scoreFlower2 = poseFactory.of(58, 130, -178.7932);
    private final Pose scoreFlower2Control1 = poseFactory.of(35, 48, 0);
    private final Pose scoreFlower2Control2 = poseFactory.of(26, 130, 0);
    private final Pose park = poseFactory.of(15, 115, 0);

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

    public Path park() {
        return line(scoreFlower2, park);
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
                        follow(follower, reverseFlower1()),
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
                        follow(follower, flower2()),
                        sequential(
                                waitMs(200),
                                intake.flowerDown()
                        )
                ),
                waitMs(flowerDelay),
                parallel(
                        follow(follower, scoreFlower2()),
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
