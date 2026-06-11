package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    Limelight3A limelight;

    private boolean isTeleOp;
    private boolean isRed;
    private int id = 0;
    private Pose3D botPose;
    private LLResult llResult;

    public enum PipelineSwitcher {
        BLUE(0), RED(1), OBELISK(2);

        private final int index;

        private PipelineSwitcher(int index) {
            this.index = index;
        }

        public int index() {
            return index;
        }
    }

    private PipelineSwitcher pipeline = PipelineSwitcher.OBELISK;

    public Limelight (boolean isTeleOp, boolean isRed) {
        this.isTeleOp = isTeleOp;
        this.isRed = isRed;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    public void switchPipeline(PipelineSwitcher newPipeline) {
        limelight.pipelineSwitch(newPipeline.index());
        pipeline = newPipeline;
    }

    public void update() {
        llResult = limelight.getLatestResult();

        switch (pipeline) {
            case BLUE:
                break;
            case RED:
                break;
            case OBELISK:
                if (isTeleOp) {
                    if (isRed) switchPipeline(PipelineSwitcher.RED);
                    else switchPipeline(PipelineSwitcher.BLUE);
                } else {

                }

                break;
        }
    }
    public int getID() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = (List<LLResultTypes.FiducialResult>) result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
        return id;
    }

    public double getTx() {
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        } else {
            return 1000;
        }
    }

    public double getTy() {
        if (llResult != null && llResult.isValid()) {
            return llResult.getTy();
        } else {
            return 1000;
        }
    }

    public double getTa() {
        if (llResult != null && llResult.isValid()) {
            return llResult.getTa();
        } else {
            return 1000;
        }
    }

    public boolean angleAlign() {
        llResult.getTx();
        return false;
    }
}
