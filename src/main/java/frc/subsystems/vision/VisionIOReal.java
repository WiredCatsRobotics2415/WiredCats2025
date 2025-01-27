package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.Subsystems.VisionConstants;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIOReal implements VisionIO {
    private String[] poseEstimationLimelightNames;

    public VisionIOReal() {
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
            PoseEstimate estimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(poseEstimationLimelightNames[i]);
            if (estimate == null) estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
        }
    }

    @Override
    public void setRobotOrientation(double yaw) {
        for (String llName : poseEstimationLimelightNames) {
            LimelightHelpers.SetRobotOrientation(llName, yaw, 0, 0, 0, 0, 0);
        }
    }
}
