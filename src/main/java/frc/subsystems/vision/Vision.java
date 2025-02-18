package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.VisionConstants;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.Util;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;
    public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    public enum EndEffectorPipeline {
        DriverView, NeuralNetwork
    }

    public enum ObjectRecognized {
        Coral, Algae
    }

    private Vision() {
        io = (VisionIO) Util.getIOImplementation(VisionIOReal.class, VisionIOSim.class, new VisionIO() {});
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
        for (int i = 0; i < 3; i++) {
            if (inputs.poseTagCounts[i] > 0) {
                averageX += inputs.poseEstimates[i].getX();
                averageY += inputs.poseEstimates[i].getY();
                usedPoses += 1;
            }
        }
        if (usedPoses == 0) return null;
        return new Pose2d(averageX / usedPoses, averageY / usedPoses, new Rotation2d());
    }

    public Pose2d getSinglePoseEstimate() {

        double x = (inputs.poseEstimates[1].getX() + inputs.poseEstimates[2].getX()) / 2;
        double y = (inputs.poseEstimates[1].getY() + inputs.poseEstimates[2].getY()) / 2;

        Pose2d singleTagPose = new Pose2d(x, y, new Rotation2d());
        return singleTagPose;

    }

    /**
     * Returns the current average rotation reported by megatag 1. Will return null if no limelights can see any tags.
     */
    public Rotation2d getCurrentAverageRotation() {
        double averageTheta = 0.0d;
        int usedPoses = 0;
        for (int i = 0; i < 3; i++) {
            if (inputs.poseTagCounts[i] > 0) {
                averageTheta += io.getMT1RotationOf(i).getDegrees();
                usedPoses += 1;
            }
        }
        if (usedPoses == 0) return null;
        return Rotation2d.fromDegrees(averageTheta / usedPoses);
    }

    public void setEndEffectorStreamOrientation(boolean upsideDown) {
        io.setEndEffectorStreamOrientation(upsideDown);
    }

    public int getEndEffectorCameraAveragePixelValue() { return inputs.endEffectorCameraAveragePixelValue; }

    public void setEndEffectorPipeline(EndEffectorPipeline pipeline) {
        io.setEndEffectorPipeline(pipeline);
    }

    public boolean objectDetected() {
        return inputs.objectDetected;
    }

    public double getObjectDetectedTx() { return inputs.detectedObjectTx; }

    public ObjectRecognized getObjectDetectedType() {
        return inputs.detectedObjectLabel == 0 ? ObjectRecognized.Algae : ObjectRecognized.Coral;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
