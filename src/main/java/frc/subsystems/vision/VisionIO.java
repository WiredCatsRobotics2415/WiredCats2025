package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.subsystems.vision.Vision.EndEffectorPipeline;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public Pose2d[] poseEstimates;
        public double[] poseTimestampsSeconds;
        public double[] poseLatencies;
        public double[] poseTagDistances;
        public int[] poseTagCounts;

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
