package frc.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.constants.Subsystems.SwerveConstants;
import frc.constants.TunerConstants;
import frc.subsystems.intake.Intake;
import frc.subsystems.vision.Vision;

/** Automatically drives towards and intakes a note. */
public class AutoNoteDetect extends Command {
    // GENERAL
    private Vision vision = Vision.getInstance();
    private Intake intake = Intake.getInstance();

    // SWERVE
    private final SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric()
        .withVelocityX(0.5 * Controls.MaxDriveMeterS);

    public AutoNoteDetect() {
        addRequirements(TunerConstants.DriveTrain);
    }

    @Override
    public void initialize() {
        intake.intakeAndWaitForNote().schedule();
    }

    @Override
    public void execute() {
        if (vision.isNoteVisible()) {
            // turns robot to current pose + x-degree
            Rotation2d pose = TunerConstants.DriveTrain.getState().Pose.getRotation();

            if (!MathUtil.isNear(0, vision.getNoteAngleOnX(),
                SwerveConstants.HeadingControllerTolerance)) {

                // System.out.println(pose);
                // System.out.println(pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())));

                TunerConstants.DriveTrain.setControl(
                    TunerConstants.DriveTrain.driveFacingAngle.withTargetDirection(
                        pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX()))));
            } else {
                TunerConstants.DriveTrain.setControl(driveForward);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        TunerConstants.DriveTrain.setControl(new SwerveRequest.SwerveDriveBrake());
        intake.off().schedule();
    }

    @Override
    public boolean isFinished() {
        // return false;
        return (intake.sensorTrigger());
    }
}
