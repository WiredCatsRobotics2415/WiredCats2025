package frc.subsystems.vision;

import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIOSim implements VisionIO {
    public VisionIOSim() {

    }

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        for (int i = 0; i < 3; i++) {
            PoseEstimate estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
        }
    }
}
