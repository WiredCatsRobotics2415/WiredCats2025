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
        PoseEstimate[] estimates = new PoseEstimate[inputs.poseEstimates.length];
        for (int i = 0; i < inputs.poseEstimates.length; i++) {
            estimates[i] = new PoseEstimate(inputs.poseEstimates[i], inputs.poseTimestampsSeconds[i],
                inputs.poseLatencies[i], inputs.poseTagCounts[i], 0.0d, 0.0d, 0.0d, null, true);
        }
        return estimates;
    }

    /**
     * @return Gets the horizontal angle returned by the intake limelight's note detection.
     */
    public double getNoteAngleOnX() { return inputs.noteAngleX; }

    public boolean isNoteVisible() { return inputs.noteVisible; }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }
}
