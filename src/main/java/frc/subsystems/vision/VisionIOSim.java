package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.Subsystems.VisionConstants;
import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIOSim implements VisionIO {
    private String[] poseEstimationLimelightNames;

    public VisionIOSim() {
        poseEstimationLimelightNames = new String[] { VisionConstants.FrontRightName, VisionConstants.FrontLeftName,
            VisionConstants.BackCenterName };
    }

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        inputs.poseEstimates = new Pose2d[poseEstimationLimelightNames.length];
        inputs.poseLatencies = new double[poseEstimationLimelightNames.length];
        inputs.poseTimestampsSeconds = new double[poseEstimationLimelightNames.length];
        inputs.poseTagCounts = new int[poseEstimationLimelightNames.length];
        for (int i = 0; i < poseEstimationLimelightNames.length; i++) {
            PoseEstimate estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
        }
    }
}
