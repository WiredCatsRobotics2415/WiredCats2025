package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    abstract class VisionIOInputs {
        public Pose2d[] poseEstimates;
        public double[] poseTimestampsSeconds;
        public double[] poseLatencies;
        public int[] poseTagCounts;
    }

    public default void updateInputs(VisionIOInputsAutoLogged inputs) {}

    public default void setRobotOrientation(double yaw) {}

    public default Rotation2d getMT1RotationOf(int index) {
        return Rotation2d.kZero;
    }

    public default void setEndEffectorStreamOrientation(boolean upsideDown) {}
}
