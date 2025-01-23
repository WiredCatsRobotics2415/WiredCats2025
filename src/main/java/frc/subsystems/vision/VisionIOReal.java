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
        PoseEstimate frontLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.FrontLeftName);
        // inputs.frontLeft_poseTimestampsSeconds = frontLeft.timestampSeconds;
        // inputs.frontLeft_poseEstimate = frontLeft.pose;
        // inputs.frontLeft_poseLatency = frontLeft.latency;
        // inputs.frontLeft_poseTagCount = frontLeft.tagCount;

        inputs.frontLeft_poseTimestampsSeconds = 0;
        inputs.frontLeft_poseEstimate = new Pose2d();
        inputs.frontLeft_poseLatency = 0;
        inputs.frontLeft_poseTagCount = 0;

        PoseEstimate frontRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.FrontRightName);
        // inputs.frontRight_poseTimestampsSeconds = frontRight.timestampSeconds;
        // inputs.frontRight_poseEstimate = frontRight.pose;
        // inputs.frontRight_poseLatency = frontRight.latency;
        // inputs.frontRight_poseTagCount = frontRight.tagCount;

        inputs.frontRight_poseTimestampsSeconds = 0;
        inputs.frontRight_poseEstimate = new Pose2d();
        inputs.frontRight_poseLatency = 0;
        inputs.frontRight_poseTagCount = 0;

        PoseEstimate backCenter = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.BackCenterName);
        // inputs.backCenter_poseTimestampsSeconds = backCenter.timestampSeconds;
        // inputs.backCenter_poseEstimate = backCenter.pose;
        // inputs.backCenter_poseLatency = backCenter.latency;
        // inputs.backCenter_poseTagCount = backCenter.tagCount;

        inputs.backCenter_poseTimestampsSeconds = 0;
        inputs.backCenter_poseEstimate = new Pose2d();
        inputs.backCenter_poseLatency = 0;
        inputs.backCenter_poseTagCount = 0;
    }

    @Override
    public void setRobotOrientation(double yaw) {
        for (String llName : poseEstimationLimelightNames) {
            LimelightHelpers.SetRobotOrientation(llName, yaw, 0, 0, 0, 0, 0);
        }
    }
}
