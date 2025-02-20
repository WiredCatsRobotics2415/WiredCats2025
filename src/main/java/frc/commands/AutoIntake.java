
package frc.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
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
        if (RobotState.isAutonomous()) {
            PPHolonomicDriveController.overrideXYFeedback(() -> {
                if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
                    double ty = vision.getObjectDetectedTy();
                    double tx = vision.getObjectDetectedTx();
                    // Temp values, can move to constants
                    double cameraHeight = 1.0; // Camera height in meters
                    double targetHeight = 0.0; // Target height in meters
                    double cameraAngle = 0.0; // Camera angle in degrees
                    double distance = (targetHeight - cameraHeight) /
                        Math.tan(Units.degreesToRadians(cameraAngle + ty));
                    Rotation2d pose = drive.getState().Pose.getRotation();

                    double objectAngle = pose.getRadians() - Units.degreesToRadians(tx);
                    double minTimeToObject = distance / Controls.MaxDriveMeterS;
                    double xFeedback = distance * Math.cos(objectAngle) / minTimeToObject;
                    return xFeedback;
                } else {
                    return 0.0;
                }
            }, () -> {
                if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
                    double ty = vision.getObjectDetectedTy();
                    double tx = vision.getObjectDetectedTx();
                    // Temp values
                    double cameraHeight = 1.0; // Camera height in meters
                    double targetHeight = 0.0; // Target height in meters
                    double cameraAngle = 0.0; // Camera angle in degrees
                    double distance = (targetHeight - cameraHeight) /
                        Math.tan(Units.degreesToRadians(cameraAngle + ty));
                    Rotation2d pose = drive.getState().Pose.getRotation();

                    double objectAngle = pose.getRadians() - Units.degreesToRadians(tx);
                    double minTimeToObject = distance / Controls.MaxDriveMeterS;
                    double yFeedback = distance * Math.sin(objectAngle) / minTimeToObject;
                    return yFeedback;
                } else {
                    return 0.0;
                }
            });
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
                    double ty = vision.getObjectDetectedTy();
                    double tx = vision.getObjectDetectedTx();
                    // Temp values
                    double cameraHeight = 1.0; // Camera height in meters
                    double targetHeight = 0.0; // Target height in meters
                    double cameraAngle = 0.0; // Camera angle in degrees
                    double distance = (targetHeight - cameraHeight) /
                        Math.tan(Units.degreesToRadians(cameraAngle + ty));
                    double minTimeToObject = distance / Controls.MaxDriveMeterS;
                    return Units.degreesToRadians(tx) / minTimeToObject;
                } else {
                    // no override if object not detected
                    return 0.0;
                }
            });
        }
    }

    @Override
    public void execute() {
        if (RobotState.isAutonomous()) return;
        if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
            double ty = vision.getObjectDetectedTy();
            double tx = vision.getObjectDetectedTx();
            // Temp values
            double cameraHeight = 1.0; // Camera height in meters
            double targetHeight = 0.0; // Target height in meters
            double cameraAngle = 0.0; // Camera angle in degrees
            double distance = (targetHeight - cameraHeight) / Math.tan(Units.degreesToRadians(cameraAngle + ty));
            double minTimeToObject = distance / Controls.MaxDriveMeterS;
            // kp used to control rotational rate of robot
            double kp = 1 / minTimeToObject;
            drive.setControl(driveForward.withRotationalRate(Units.degreesToRadians(tx) * kp));
        }
    }

    @Override
    public boolean isFinished() {
        PPHolonomicDriveController.clearXYFeedbackOverride();
        PPHolonomicDriveController.clearRotationFeedbackOverride();
        return endEffector.hasCoral();
    }
}
