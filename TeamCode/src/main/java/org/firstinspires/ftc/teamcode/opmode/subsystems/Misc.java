package org.firstinspires.ftc.teamcode.opmode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.opmode.subsystems.Constants.*;

public class Misc {
    public HardwareMap hwmap;
    public Limelight3A limelight;

    public Misc(HardwareMap hwmap){
        this.hwmap = hwmap;
    }

    public boolean canShoot(String alliance){
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (alliance.equalsIgnoreCase("red") && id >= 30 && id <= 37) return true;
            if (alliance.equalsIgnoreCase("blue") && id >= 38 && id <= 45) return true;
        }
        return false;
    }

    public void init(){
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void initLoop(){

    }

    public void start(){
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public void update(){

    }
}
