
package frc.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.constants.Subsystems.DriveAutoConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.vision.Vision;
import frc.subsystems.vision.Vision.ObjectRecognized;

public class AutoIntake extends Command {
    private Vision vision = Vision.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();

    private final SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric()
        .withVelocityX(0.5 * Controls.MaxDriveMeterS);

    public AutoIntake() {
        addRequirements(drive, endEffector);
    }

    @Override
    public void initialize() {
        endEffector.intakeAndWaitForCoral().schedule();
    }

    @Override
    public void execute() {
        if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
            // turns robot to current pose + x-degree
            Rotation2d pose = drive.getState().Pose.getRotation();

            double tx = vision.getObjectDetectedTx();
            if (!MathUtil.isNear(0, tx, DriveAutoConstants.HeadingTolerance)) {

                // System.out.println(pose);
                // System.out.println(pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())));

                drive.setControl(drive.driveToPositionFacingAngleRequest
                    .withTargetDirection(pose.minus(Rotation2d.fromDegrees(tx))));
            } else {
                drive.setControl(driveForward);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // return false;
        return endEffector.hasCoral();
    }
}
