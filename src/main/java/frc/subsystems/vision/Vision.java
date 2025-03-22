package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Measurements;
import frc.constants.Subsystems.VisionConstants;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.Util;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    public enum EndEffectorPipeline {
        DriverView, NeuralNetwork
    }

    public enum ObjectRecognized {
        Coral, Algae
    }

    private Vision() {
        io = (VisionIO) Util.getIOImplementation(VisionIOReal.class, VisionIOSim.class, new VisionIO() {});
        setEndEffectorPipeline(EndEffectorPipeline.DriverView);
        setEndEffectorStreamOrientation(Arm.getInstance().getMeasurement() > 90);

        // Note: this is a pretty costly function to be running on the rio constantly
        // DashboardManager.getInstance().addBoolSupplier(true, "Reef ST available", () -> {
        // return aLimelightCanSeeOneOf(LimelightsForElements.Reef, ReefMeasurements.reefIds.get());
        // });
    }

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    public void sendOrientation(double orientationDegrees, double yawRateDegS) {
        io.setRobotOrientation(orientationDegrees, yawRateDegS);
    }

    public PoseEstimate[] getPoseEstimates() {
        PoseEstimate[] estimates = new PoseEstimate[VisionConstants.PoseEstimationLLNames.length];

        for (int i = 0; i < estimates.length; i++) {
            estimates[i] = new PoseEstimate(inputs.poseEstimates[i], inputs.poseTimestampsSeconds[i],
                inputs.poseLatencies[i], inputs.poseTagCounts[i], 0, 0, 0, null, true);
        }

        return estimates;
    }

    /** Averages together the pose from all apriltag limelights and returns that average, with NO ROTATION component */
    public Pose2d getCurrentAveragePose() {
        double averageX = 0.0d, averageY = 0.0d;
        int usedPoses = 0;
        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            if (inputs.poseTagCounts[i] > 0) {
                averageX += inputs.poseEstimates[i].getX();
                averageY += inputs.poseEstimates[i].getY();
                usedPoses += 1;
            }
        }
        if (usedPoses == 0) return null;
        return new Pose2d(averageX / usedPoses, averageY / usedPoses, Rotation2d.kZero);
    }

    public PoseEstimate getSingleTagPoseEstimate(LimelightsForElements limelights, int tag) {
        PoseEstimate pe = new PoseEstimate();

        double xSum = 0;
        double ySum = 0;
        double smallestTimestamp = Double.MAX_VALUE;
        int numCamerasUsed = 0;
        for (int index : limelights.indexInPEList) {
            if (inputs.poseTagCounts[index] > 0 && inputs.nearestTags[index] == tag) {
                xSum += inputs.poseEstimates[index].getX();
                ySum += inputs.poseEstimates[index].getY();
                numCamerasUsed += 1;
                smallestTimestamp = Math.min(smallestTimestamp, inputs.poseTimestampsSeconds[index]);
            }
        }

        if (numCamerasUsed == 0) return null;
        pe.pose = new Pose2d(xSum / numCamerasUsed, ySum / numCamerasUsed, Rotation2d.kZero);
        // Choosing latest pose increases trust
        pe.timestampSeconds = smallestTimestamp;

        return pe;
    }

    /**
     * Finds the nearest apriltag id to each limelight. If neither can see a tag: -1. If one can see a tag: return that tag. If both can see different tags: returns tag that robot is currently closer to If both can see the same tag: return that tag. Assumes the limelights index list is either 1 or 2 elements long.
     */
    public int nearestTagToAtLeastOneOf(LimelightsForElements limelights) {
        int[] nearestTagToEach = new int[limelights.indexInPEList.length];
        int llCounter = 0;
        for (int indexInPE : limelights.indexInPEList) {
            nearestTagToEach[llCounter] = inputs.nearestTags[indexInPE];
            llCounter += 1;
        }

        if (nearestTagToEach.length == 1) {
            return nearestTagToEach[0];
        }

        System.out.println("nearestTagToAtLeastOneOf: " + nearestTagToEach[0] + ", " + nearestTagToEach[1]);
        if (nearestTagToEach[0] == -1 || nearestTagToEach[1] == -1) {
            if (nearestTagToEach[0] == nearestTagToEach[1]) {
                return -1;
            }
            return nearestTagToEach[0] == -1 ? nearestTagToEach[1] : nearestTagToEach[0];
        } else {
            if (nearestTagToEach[0] == nearestTagToEach[1]) return nearestTagToEach[0];
            System.out.println(
                "nearestTagToAtLeastOneOf: debating between " + nearestTagToEach[0] + " and " + nearestTagToEach[1]);
            return whichTagCloserToCurrentRobotPosition(nearestTagToEach[0], nearestTagToEach[1]) ? nearestTagToEach[0]
                : nearestTagToEach[1];
        }
    }

    /**
     * True if tag 1 is closer to the current robot position, false if tag 2 is closer
     */
    private boolean whichTagCloserToCurrentRobotPosition(int tag1, int tag2) {
        Pose2d tag1Pose = Measurements.ApriltagFieldLayout.getTagPose(tag1).get().toPose2d();
        Pose2d tag2Pose = Measurements.ApriltagFieldLayout.getTagPose(tag2).get().toPose2d();
        Pose2d currentRobotPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
        return tag1Pose.getTranslation().getDistance(currentRobotPose.getTranslation()) < tag2Pose.getTranslation()
            .getDistance(currentRobotPose.getTranslation());
    }

    /**
     * True if one of the limelights can see one of the tags
     */
    public boolean aLimelightCanSeeOneOf(LimelightsForElements limelights, int[] tagSet) {
        int nearestTagToEach = nearestTagToAtLeastOneOf(limelights);
        for (int tagId : tagSet) {
            if (nearestTagToEach == tagId) return true;
        }
        return false;
    }

    /**
     * Returns the current average rotation reported by megatag 1. Will return null if no limelights can see any tags.
     */
    public Rotation2d getCurrentAverageRotation() {
        double averageTheta = 0.0d;
        int usedPoses = 0;
        for (int i = 0; i < VisionConstants.PoseEstimationLLNames.length; i++) {
            if (inputs.poseTagCounts[i] > 0) {
                averageTheta += io.getMT1RotationOf(i).getDegrees();
                usedPoses += 1;
            }
        }
        if (usedPoses == 0) return null;
        return Rotation2d.fromDegrees(averageTheta / usedPoses);
    }

    public int getClosestApriltagTo(int peCameraIdx) {
        return inputs.nearestTags[peCameraIdx];
    }

    public void setEndEffectorStreamOrientation(boolean upsideDown) {
        io.setEndEffectorStreamOrientation(upsideDown);
    }

    public int getEndEffectorCameraAveragePixelValue() { return inputs.endEffectorCameraAveragePixelValue; }

    public void setEndEffectorPipeline(EndEffectorPipeline pipeline) {
        System.out.println("Set EE pipeline to: " + pipeline);
        io.setEndEffectorPipeline(pipeline);
    }

    public boolean objectDetected() {
        return inputs.objectDetected;
    }

    public double getObjectDetectedTx() { return inputs.detectedObjectTx; }

    public double getObjectDetectedTy() { return inputs.detectedObjectTy; }

    public ObjectRecognized getObjectDetectedType() {
        return inputs.detectedObjectLabel == 0 ? ObjectRecognized.Algae : ObjectRecognized.Coral;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
