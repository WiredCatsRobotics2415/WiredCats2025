package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.Subsystems.VisionConstants;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.PoseEstimate;

public class VisionIOReal implements VisionIO {
    public VisionIOReal() {
        
    }

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        inputs.poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            PoseEstimate estimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.PoseEstimationLLNames[i]);
            if (estimate == null) estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
        }
    }

    @Override
    public void setRobotOrientation(double yaw) {
        for (String llName : VisionConstants.PoseEstimationLLNames) {
            LimelightHelpers.SetRobotOrientation(llName, yaw, 0, 0, 0, 0, 0);
        }
    }

    @Override
    public void setEndEffectorStreamOrientation(boolean upsideDown) {
        LimelightHelpers.setPythonScriptData(VisionConstants.FrontLeftName, new double[] { upsideDown ? 1.0d : -1.0d });
    }
}
