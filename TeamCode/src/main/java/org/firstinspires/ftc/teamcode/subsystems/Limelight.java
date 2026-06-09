package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.sun.tools.javac.util.List;

public class Limelight {
    Limelight3A limelight;

    private boolean isTeleOp;
    private boolean isRed;
    private int id = 0;

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
}
