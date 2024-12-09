package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LimelightHelpers.PoseEstimate;
import frc.util.Utils;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    private Vision() {
        io = (VisionIO) Utils.getIOImplementation(VisionIOReal.class, VisionIOSim.class,
            VisionIO.class);
    }

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    public void sendOrientation(Rotation2d orientation) {
        io.setRobotOrientation(orientation.getDegrees());
    }

    public PoseEstimate getShooterResults() {
        return new PoseEstimate(inputs.poseEstimate, inputs.poseTimestampSeconds,
            inputs.poseLatency, inputs.poseTagCount, 0.0d, 0.0d, 0.0d, null);
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
