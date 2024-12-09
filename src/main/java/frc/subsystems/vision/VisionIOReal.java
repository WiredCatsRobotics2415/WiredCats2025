package frc.subsystems.vision;

import frc.constants.Subsystems.VisionConstants;
import frc.util.LimelightHelpers;
import frc.util.LimelightHelpers.PoseEstimate;

public class VisionIOReal implements VisionIO {
    public VisionIOReal() {}

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        PoseEstimate poseEstimate = LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.ShooterLimelightName);
        if (poseEstimate == null) {
            return;
        }
        inputs.poseEstimate = poseEstimate.pose;
        inputs.poseTimestampSeconds = poseEstimate.timestampSeconds;
        inputs.poseLatency = poseEstimate.latency;
        inputs.poseTagCount = poseEstimate.tagCount;

        inputs.noteAngleX = LimelightHelpers.getTX(VisionConstants.IntakeLimelightName);
        inputs.noteVisible = LimelightHelpers
            .getTA(VisionConstants.IntakeLimelightName) != 0;
    }

    @Override
    public void setRobotOrientation(double yaw) {
        LimelightHelpers.SetRobotOrientation(VisionConstants.ShooterLimelightName, yaw, 0,
            0, 0, 0, 0);
    }
}
