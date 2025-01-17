package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.Subsystems.VisionConstants;
import frc.utils.LimelightHelpers;

public class VisionIOReal implements VisionIO {
    String[] poseEstimationLimelightNames;

    public VisionIOReal() {
        poseEstimationLimelightNames = new String[] { VisionConstants.FirstLL3GName, VisionConstants.SecondLL3GName,
            VisionConstants.ThirdLL3GName };
    }

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        /*
         * inputs.poseEstimates = new Pose2d[poseEstimationLimelightNames.length]; inputs.poseTimestampsSeconds = new double[poseEstimationLimelightNames.length]; inputs.poseLatencies = new double[poseEstimationLimelightNames.length]; inputs.poseTagCounts = new int[poseEstimationLimelightNames.length];
         *
         * for (int i = 0; i < poseEstimationLimelightNames.length; i++) { PoseEstimate poseEstimate = LimelightHelpers .getBotPoseEstimate_wpiBlue_MegaTag2(poseEstimationLimelightNames[i]); if (poseEstimate == null) continue; inputs.poseEstimates[i] = poseEstimate.pose; inputs.poseTimestampsSeconds[i] = poseEstimate.timestampSeconds; inputs.poseLatencies[i] = poseEstimate.latency; inputs.poseTagCounts[i] = poseEstimate.tagCount; }
         */
        inputs.poseEstimates = new Pose2d[0];
        inputs.poseLatencies = new double[0];
        inputs.poseTagCounts = new int[0];
        inputs.poseTimestampsSeconds = new double[0];
    }

    @Override
    public void setRobotOrientation(double yaw) {
        for (String llName : poseEstimationLimelightNames) {
            LimelightHelpers.SetRobotOrientation(llName, yaw, 0, 0, 0, 0, 0);
        }
    }
}
