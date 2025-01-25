package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOSim implements VisionIO {
    public VisionIOSim() {

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.frontLeft_poseTimestampsSeconds = 0.0d;
        inputs.frontLeft_poseEstimate = new Pose2d();
        inputs.frontLeft_poseLatency = 0.0d;
        inputs.frontLeft_poseTagCount = 0;

        inputs.frontRight_poseTimestampsSeconds = 0.0d;
        inputs.frontRight_poseEstimate = new Pose2d();
        inputs.frontRight_poseLatency = 0.0d;
        inputs.frontRight_poseTagCount = 0;

        inputs.backCenter_poseTimestampsSeconds = 0.0d;
        inputs.backCenter_poseEstimate = new Pose2d();
        inputs.backCenter_poseLatency = 0.0d;
        inputs.backCenter_poseTagCount = 0;
    }
}
