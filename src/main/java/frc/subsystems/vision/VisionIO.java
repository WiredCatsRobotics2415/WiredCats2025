package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.vision.Vision.EndEffectorPipeline;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public Pose2d[] poseEstimates = new Pose2d[VisionConstants.PoseEstimationLLNames.length];
        public double[] poseTimestampsSeconds = new double[VisionConstants.PoseEstimationLLNames.length];
        public double[] poseLatencies = new double[VisionConstants.PoseEstimationLLNames.length];
        public double[] poseTagDistances = new double[VisionConstants.PoseEstimationLLNames.length];
        public int[] poseTagCounts = new int[VisionConstants.PoseEstimationLLNames.length];
        public int[] nearestTags = new int[VisionConstants.PoseEstimationLLNames.length];

        public int endEffectorCameraAveragePixelValue;
        public boolean objectDetected;
        public double detectedObjectTx;
        public int detectedObjectLabel;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setRobotOrientation(double yaw, double yawRate) {}

    public default Rotation2d getMT1RotationOf(int index) {
        return Rotation2d.kZero;
    }

    public default void setEndEffectorStreamOrientation(boolean upsideDown) {}

    public default void setEndEffectorPipeline(EndEffectorPipeline pipeline) {};
}
