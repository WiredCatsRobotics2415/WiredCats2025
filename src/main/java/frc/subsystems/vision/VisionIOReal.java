package frc.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.vision.Vision.EndEffectorPipeline;
import frc.utils.LimelightHelpers;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.LimelightHelpers.RawDetection;
import frc.utils.LimelightHelpers.RawFiducial;
import frc.utils.math.Algebra;

public class VisionIOReal implements VisionIO {
    private EndEffectorPipeline currentPipeline;

    public VisionIOReal() {
        Transform3d fl = RobotMeasurements.FrontLeftCamera;
        LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.FrontLeftName, fl.getX(), fl.getY(), fl.getZ(),
            fl.getRotation().getMeasureX().in(Degrees), fl.getRotation().getMeasureY().in(Degrees),
            fl.getRotation().getMeasureZ().in(Degrees));

        Transform3d fr = RobotMeasurements.FrontRightCamera;
        LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.FrontRightName, fr.getX(), fr.getY(), fr.getZ(),
            fr.getRotation().getMeasureX().in(Degrees), fr.getRotation().getMeasureY().in(Degrees),
            fr.getRotation().getMeasureZ().in(Degrees));

        Transform3d b = RobotMeasurements.BackCamera;
        LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.BackCenterName, b.getX(), b.getY(), b.getZ(),
            b.getRotation().getMeasureX().in(Degrees), b.getRotation().getMeasureY().in(Degrees),
            b.getRotation().getMeasureZ().in(Degrees));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];
        inputs.poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];
        inputs.nearestTags = new int[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            PoseEstimate estimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.PoseEstimationLLNames[i]);
            if (estimate == null) {
                estimate = PoseEstimate.zero;
            }
            inputs.poseEstimates[i] = estimate.pose;
            inputs.poseLatencies[i] = estimate.latency;
            inputs.poseTimestampsSeconds[i] = estimate.timestampSeconds;
            inputs.poseTagCounts[i] = estimate.tagCount;
            inputs.poseTagDistances[i] = estimate.avgTagDist;

            double nearestTagLength = Double.MAX_VALUE;
            int nearestTagId = -1;
            for (RawFiducial tag : estimate.rawFiducials) {
                if (tag.distToCamera < nearestTagLength) {
                    nearestTagLength = tag.distToCamera;
                    nearestTagId = tag.id;
                }
            }
            inputs.nearestTags[i] = nearestTagId;
        }

        if (currentPipeline == EndEffectorPipeline.DriverView) {
            double[] endEffectorData = LimelightHelpers.getPythonScriptData(VisionConstants.EndEffectorName);
            if (endEffectorData.length > 0) inputs.endEffectorCameraAveragePixelValue = (int) endEffectorData[0];
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
                    double closeness = Math.signum(objectsDetected[i].tync) *
                        Algebra.euclideanDistance(objectsDetected[i].txnc, objectsDetected[i].tync);
                    if (closeness < closestValue) {
                        closestValue = closeness;
                        closestIndex = i;
                    }
                }
                object = objectsDetected[closestIndex];
            }

            inputs.detectedObjectTx = object.txnc;
            inputs.detectedObjectTy = object.tync;
            inputs.detectedObjectLabel = object.classId;

            inputs.detectedObjectBB_Corner1x = object.corner0_X;
            inputs.detectedObjectBB_Corner1y = object.corner0_Y;
            inputs.detectedObjectBB_Corner2x = object.corner1_X;
            inputs.detectedObjectBB_Corner2y = object.corner1_Y;
            inputs.detectedObjectBB_Corner3x = object.corner2_X;
            inputs.detectedObjectBB_Corner3y = object.corner2_Y;
            inputs.detectedObjectBB_Corner4x = object.corner3_X;
            inputs.detectedObjectBB_Corner4y = object.corner3_Y;
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
