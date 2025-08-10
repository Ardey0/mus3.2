package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private Limelight3A limelight = null;
    public final int    RED_YELLOW = 0,
                        BLUE_YELLOW = 1,
                        RED = 4,
                        BLUE = 3,
                        YELLOW = 6,
                        RED_YELLOW_EXPERIMENTAL = 7;


    public Limelight(@NonNull HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(100); // 1 <= hz <= 250
        this.limelight.start();
    }

    public void switchPipeline(int pipeline) {
        // dureaza cateva ms sa se schimbe pipelineul dar programul nu asteapta
        limelight.pipelineSwitch(pipeline);
    }

    public double getTargetTx() {
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getTx();
        }
        return 0;
    }

    public double getTargetTy() {
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getTy();
        }
        return 0;
    }

    public double getTargetArea() {
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getTa();
        }
        return 0;
    }

    public double getAngle(){
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            return result.getPythonOutput()[5];
        }
        return 0;
    }

    public long getStaleness() {
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            return result.getStaleness();
        }
        return -1;
    }
}
