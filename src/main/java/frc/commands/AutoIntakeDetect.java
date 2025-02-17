
package frc.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.vision.Vision;

public class AutoIntakeDetect extends Command {
    private Vision vision = Vision.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    private CommandSwerveDrivetrain CommandSwerveDriveTrain = CommandSwerveDrivetrain.getInstance();

    private final SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric()
        .withVelocityX(0.5 * Controls.MaxDriveMeterS);

    public AutoIntakeDetect() {
        addRequirements(CommandSwerveDriveTrain);
    }

    @Override
    public void initialize() {
        endEffector.intakeAndWaitForCoral().schedule();
    }

    // @Override
    // public void execute() {
    // if (vision.isCoralVisible()) {
    // // turns robot to current pose + x-degree
    // Rotation2d pose = CommandSwerveDriveTrain.getState().Pose.getRotation();

    // if (!MathUtil.isNear(0, vision.getCoralAngleOnX(),
    // DriveAutoConstants.HeadingTolerance)) {

    // // System.out.println(pose);
    // // System.out.println(pose.minus(Rotation2d.fromDegrees(vision.getNoteAngleOnX())));

    // CommandSwerveDriveTrain.setControl(
    // CommandSwerveDriveTrain.driveToPositionFacingAngleRequest.withTargetDirection(
    // pose.minus(Rotation2d.fromDegrees(vision.getCoralAngleOnX()))));
    // } else {
    // CommandSwerveDriveTrain.setControl(driveForward);
    // }
    // }
    // }

    @Override
    public void end(boolean interrupted) {
        CommandSwerveDriveTrain.setControl(new SwerveRequest.SwerveDriveBrake());
        endEffector.turnOff().schedule();
    }

    @Override
    public boolean isFinished() {
        // return false;
        return (endEffector.hasCoral());
    }
}
