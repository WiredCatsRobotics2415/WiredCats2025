package frc.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Measurements;
import frc.constants.Subsystems.SwerveConstants;
import frc.constants.TunerConstants;

/**
 * The "FixAll" preset (FIX ALL subsystems to their ideal position for scoring). Instance
 * can be reused (ie. you can construct this command once for a button binding).
 * Automatically compensates for alliance.
 */
public class FixAll extends Command {
    private Rotation2d goalHeading;

    public FixAll() {
        addRequirements(TunerConstants.DriveTrain); // TODO: add other subsystems
    }

    @Override
    public void initialize() {
        Translation2d speakerDist = Measurements.getSpeakerLocation()
            .minus(TunerConstants.DriveTrain.getState().Pose.getTranslation());
        goalHeading = Rotation2d
            .fromRadians(Math.atan2(speakerDist.getY(), speakerDist.getX()))
            .plus(Rotation2d.fromDegrees(180));
    }

    @Override
    public void execute() {
        // Heading alignment
        // let (x, y) be the difference of position between the speaker and the robot
        // swerveheading = (arctan(y/x))

        TunerConstants.DriveTrain.setControl(
            TunerConstants.DriveTrain.driveFacingAngle.withTargetDirection(goalHeading));
    }

    @Override
    public boolean isFinished() {
        double currentRotation = TunerConstants.DriveTrain.getState().Pose.getRotation()
            .getDegrees();
        return MathUtil.isNear(goalHeading.getDegrees(), currentRotation,
            SwerveConstants.HeadingControllerTolerance);
    }
}
