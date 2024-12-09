package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    abstract class VisionIOInputs {
        public double noteAngleX;
        public boolean noteVisible;

        public Pose2d poseEstimate;
        public double poseTimestampSeconds;
        public double poseLatency;
        public int poseTagCount;
    }

    public default void updateInputs(VisionIOInputsAutoLogged inputs) {}

    public default void setRobotOrientation(double yaw) {}
}
