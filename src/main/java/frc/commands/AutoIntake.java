
package frc.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.vision.Vision;
import frc.subsystems.vision.Vision.EndEffectorPipeline;
import frc.subsystems.vision.Vision.ObjectRecognized;
import frc.utils.tuning.TuneableNumber;

public class AutoIntake extends Command {
    private Vision vision;
    private EndEffector endEffector;
    private CommandSwerveDrivetrain drive;

    private TuneableNumber hasSeenCoralSeconds;

    private SwerveRequest.RobotCentric driveBackward;

    private boolean hasSeenCoral;
    public Timer countSinceLastSeenCoral;
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void initialize() {
        hasSeenCoral = false;
        driveHeading.HeadingController = new PhoenixPIDController(0.5, 0, 0.3);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        countSinceLastSeenCoral = new Timer();
        hasSeenCoralSeconds = new TuneableNumber(0.5, "AutoIntaking/hasSeenCoralSeconds");
        vision = Vision.getInstance();
        endEffector = EndEffector.getInstance();
        drive = CommandSwerveDrivetrain.getInstance();
        addRequirements(drive);
        driveBackward = new SwerveRequest.RobotCentric().withVelocityX(-0.4 * Controls.MaxDriveMeterS); // back of robot = intaking side
        RobotStatus.setRobotStateOnce(RobotState.AutoGroundIntaking);
        vision.setEndEffectorPipeline(Vision.EndEffectorPipeline.NeuralNetwork);
        endEffector.intakeAndWaitForCoral().schedule();
    }

    @Override
    public void execute() {
        if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
            if (!hasSeenCoral) {
                System.out.println("Seen a coral");
                countSinceLastSeenCoral.stop();
                countSinceLastSeenCoral.reset();
                hasSeenCoral = true;
            }
            double tx = vision.getObjectDetectedTx();
            double tolerance = 3.4;

            if (tx > tolerance || tx < -tolerance) {
                System.out.println(tx);
                System.out.println("tx outside of tolerance");
                drive.setControl(driveHeading
                    .withTargetDirection(drive.getState().Pose.getRotation().minus(Rotation2d.fromDegrees(-tx))));
            } else {
                drive.setControl(driveBackward);
            }
        } else {
            if (hasSeenCoral) {
                if (DriverStation.isAutonomous()) {
                    PPHolonomicDriveController.clearXYFeedbackOverride();
                    PPHolonomicDriveController.clearRotationFeedbackOverride();
                }
                System.out.println("lost sight of coral");
                countSinceLastSeenCoral.start();
                hasSeenCoral = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        countSinceLastSeenCoral.reset();
        countSinceLastSeenCoral.stop();
        vision.setEndEffectorPipeline(EndEffectorPipeline.DriverView);
        System.out.println("ended" + interrupted);
    }

    @Override
    public boolean isFinished() {
        return endEffector.hasCoral() || countSinceLastSeenCoral.hasElapsed(hasSeenCoralSeconds.get());
    }
}
