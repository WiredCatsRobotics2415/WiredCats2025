package frc.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.constants.TunerConstants;
import frc.robot.Robot;
import frc.sim.SimulatedEnvironment;
import frc.subsystems.drive.SwerveDrive;

public class VisionIOSim implements VisionIO {
    private SwerveDrive drive;
    private SimulatedEnvironment simEnv = Robot.getSimEnv();

    public VisionIOSim() {
        drive = TunerConstants.DriveTrain;
    }

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
        if (drive == null)
            inputs.poseEstimate = new Pose2d();
        else
            inputs.poseEstimate = drive.getState().Pose;
        inputs.poseLatency = 0.0d;
        inputs.poseTimestampSeconds = Timer.getFPGATimestamp();

        double[] simEnvResults = simEnv.getCurrentNoteTargetInfo();
        inputs.noteAngleX = simEnvResults[1];
        inputs.noteVisible = simEnvResults[0] > 0;
    }
}
