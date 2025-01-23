package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    abstract class VisionIOInputs {
        public Pose2d frontLeft_poseEstimate;
        public double frontLeft_poseTimestampsSeconds;
        public double frontLeft_poseLatency;
        public int frontLeft_poseTagCount;

        public Pose2d frontRight_poseEstimate;
        public double frontRight_poseTimestampsSeconds;
        public double frontRight_poseLatency;
        public int frontRight_poseTagCount;

        public Pose2d backCenter_poseEstimate;
        public double backCenter_poseTimestampsSeconds;
        public double backCenter_poseLatency;
        public int backCenter_poseTagCount;
    }

    public default void updateInputs(VisionIOInputsAutoLogged inputs) {}

    public default void setRobotOrientation(double yaw) {}
}
