package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.vision.Vision.EndEffectorPipeline;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.LimelightHelpers.RawDetection;
import frc.utils.math.Algebra;

public class VisionIOReal implements VisionIO {
    private EndEffectorPipeline currentPipeline;

    public VisionIOReal() {

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            PoseEstimate estimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.PoseEstimationLLNames[i]);
            if (estimate == null) estimate = PoseEstimate.zero;
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
            inputs.poseTagDistances[i] = estimate.avgTagDist;
        }

        if (currentPipeline == EndEffectorPipeline.DriverView) {
            inputs.endEffectorCameraAveragePixelValue = (int) LimelightHelpers
                .getPythonScriptData(VisionConstants.EndEffectorName)[0];
        } else {
            RawDetection[] objectsDetected = LimelightHelpers.getRawDetections(VisionConstants.EndEffectorName);
            if (objectsDetected.length == 0) {
                inputs.objectDetected = false;
                return;
            }
            inputs.objectDetected = true;

            RawDetection object = objectsDetected[0];
            if (objectsDetected.length > 1) {
                double closestValue = Double.MAX_VALUE;
                int closestIndex = 0;
                for (int i = 0; i < objectsDetected.length; i++) {
                    double closeness = Algebra.euclideanDistance(objectsDetected[i].txnc, objectsDetected[i].tync);
                    if (closeness < closestValue) {
                        closestValue = closeness;
                        closestIndex = i;
                    }
                }
                object = objectsDetected[closestIndex];
            }

            inputs.detectedObjectTx = object.txnc;
            inputs.detectedObjectLabel = object.classId;
        }
    }

    @Override
    public void setRobotOrientation(double yaw, double yawRate) {
        for (String llName : VisionConstants.PoseEstimationLLNames) {
            LimelightHelpers.SetRobotOrientation(llName, yaw, yawRate, 0, 0, 0, 0);
        }
    }

    @Override
    public Rotation2d getMT1RotationOf(int index) {
        Rotation2d rotation = LimelightHelpers
            .getBotPoseEstimate_wpiBlue(VisionConstants.PoseEstimationLLNames[index]).pose.getRotation();
        return rotation;
    }

    @Override
    public void setEndEffectorStreamOrientation(boolean upsideDown) {
        LimelightHelpers.setPythonScriptData(VisionConstants.EndEffectorName,
            new double[] { upsideDown ? 1.0d : -1.0d });
    }

    @Override
    public void setEndEffectorPipeline(EndEffectorPipeline pipeline) {
        currentPipeline = pipeline;
        LimelightHelpers.setPipelineIndex(VisionConstants.EndEffectorName,
            pipeline == EndEffectorPipeline.DriverView ? 0 : 1);
    }
}
