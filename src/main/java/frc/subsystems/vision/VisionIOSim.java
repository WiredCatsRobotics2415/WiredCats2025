package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOSim implements VisionIO {
    public VisionIOSim() {

    }

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        inputs.poseEstimates = new Pose2d[0];
        inputs.poseLatencies = new double[0];
        inputs.poseTagCounts = new int[0];
        inputs.poseTimestampsSeconds = new double[0];
    }
}
