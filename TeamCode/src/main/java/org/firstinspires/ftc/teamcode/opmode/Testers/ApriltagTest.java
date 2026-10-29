package org.firstinspires.ftc.teamcode.opmode.Testers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Apriltag Test", group = "Test")
public class ApriltagTest extends OpMode {
    Limelight3A limelight;
    String alliance = "red";

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad1.x) alliance = "blue";
        if (gamepad1.b) alliance = "red";

        boolean redSeen = false;
        boolean blueSeen = false;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id >= 30 && id <= 37) redSeen = true;
                if (id >= 38 && id <= 45) blueSeen = true;
                telemetry.addData("Tag", id);
            }
        }

        telemetry.addData("Alliance", alliance);
        if (redSeen) telemetry.addLine("Red isn't tipped");
        if (blueSeen) telemetry.addLine("Blue isn't tipped");
        if (!redSeen && !blueSeen) telemetry.addLine("Both hive cells are tipped");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
