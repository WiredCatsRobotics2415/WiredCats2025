package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.Utils;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    private Vision() {
        io = (VisionIO) Utils.getIOImplementation(VisionIOReal.class, VisionIOSim.class, VisionIO.class);
    }

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    public void sendOrientation(Rotation2d orientation) {
        io.setRobotOrientation(orientation.getDegrees());
    }

    public PoseEstimate[] getPoseEstimates() {
        PoseEstimate[] estimates = new PoseEstimate[3];

        estimates[0] = new PoseEstimate(inputs.frontLeft_poseEstimate, inputs.frontLeft_poseTimestampsSeconds,
            inputs.frontLeft_poseLatency, inputs.frontLeft_poseTagCount, 0.0d, 0.0d, 0.0d, null, true);
        estimates[1] = new PoseEstimate(inputs.frontRight_poseEstimate, inputs.frontRight_poseTimestampsSeconds,
            inputs.frontRight_poseLatency, inputs.frontRight_poseTagCount, 0.0d, 0.0d, 0.0d, null, true);
        estimates[2] = new PoseEstimate(inputs.backCenter_poseEstimate, inputs.backCenter_poseTimestampsSeconds,
            inputs.backCenter_poseLatency, inputs.backCenter_poseTagCount, 0.0d, 0.0d, 0.0d, null, true);

        return estimates;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
