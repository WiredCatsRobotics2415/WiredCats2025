package frc.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveIO {
    @AutoLog
    abstract class SwerveDriveInputs {
        public Pose2d odometryPose;

        public SwerveModuleState[] goalStates;
        public SwerveModuleState[] actualStates;
    }
}
