
package frc.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.constants.Measurements.CoralMeasurements;
import frc.constants.Measurements.RobotMeasurements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.vision.Vision;
import frc.subsystems.vision.Vision.ObjectRecognized;
import frc.utils.math.Trig;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuneableTime;

public class AutoIntake extends Command {
    private Vision vision = Vision.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();

    private TuneableNumber hasSeenCoralSeconds = new TuneableNumber(0.5, "AutoIntaking/hasSeenCoralSeconds");

    private final SwerveRequest.RobotCentric driveBackward = new SwerveRequest.RobotCentric()
        .withVelocityX(-0.5 * Controls.MaxDriveMeterS); // back of robot = intaking side

    private boolean hasSeenCoral = true;
    private Timer countSinceLastSeenCoral = new Timer();

    public AutoIntake() {
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.deadlineFor(RobotStatus.keepStateUntilInterrupted(RobotState.AutoGroundIntaking));
        vision.setEndEffectorPipeline(Vision.EndEffectorPipeline.NeuralNetwork);
        endEffector.intakeAndWaitForCoral().schedule();
        if (DriverStation.isAutonomous()) {
            PPHolonomicDriveController.overrideXYFeedback(() -> {
                if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
                    double ty = vision.getObjectDetectedTy();
                    double tx = vision.getObjectDetectedTx();

                    double distance = (CoralMeasurements.HeightFromCenterOffGround
                        .minus(RobotMeasurements.EECamHeightOffGround).in(Meters)) /
                        Math.tan(Units.degreesToRadians(RobotMeasurements.EECamForward.in(Radians) + ty));
                    Rotation2d pose = drive.getState().Pose.getRotation();

                    double objectAngle = pose.getRadians() - Units.degreesToRadians(tx);
                    double minTimeToObject = distance / Controls.MaxDriveMeterS;
                    double xFeedback = distance * Trig.cosizzle(objectAngle) / minTimeToObject;
                    return xFeedback;
                } else {
                    return 0.0;
                }
            }, () -> {
                if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
                    double ty = vision.getObjectDetectedTy();

                    double distance = (CoralMeasurements.HeightFromCenterOffGround
                        .minus(RobotMeasurements.EECamHeightOffGround).in(Meters)) /
                        Math.tan(Units.degreesToRadians(RobotMeasurements.EECamForward.in(Radians) + ty));
                    Rotation2d pose = drive.getState().Pose.getRotation();

                    double objectAngle = pose.getRadians() - Units.degreesToRadians(ty);
                    double minTimeToObject = distance / Controls.MaxDriveMeterS;
                    double yFeedback = distance * Trig.sizzle(objectAngle) / minTimeToObject;
                    return yFeedback;
                } else {
                    return 0.0;
                }
            });
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
                    double ty = vision.getObjectDetectedTy();
                    double tx = vision.getObjectDetectedTx();

                    double distance = (CoralMeasurements.HeightFromCenterOffGround
                        .minus(RobotMeasurements.EECamHeightOffGround).in(Meters)) /
                        Math.tan(Units.degreesToRadians(RobotMeasurements.EECamForward.in(Radians) + ty));

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
        if (DriverStation.isAutonomous()) return;
        if (vision.objectDetected() && vision.getObjectDetectedType() == ObjectRecognized.Coral) {
            if (!hasSeenCoral) {
                System.out.println("Seen a coral");
                countSinceLastSeenCoral.stop();
                countSinceLastSeenCoral.reset();
                hasSeenCoral = true;
            }
            double ty = vision.getObjectDetectedTy();
            double tx = vision.getObjectDetectedTx();

            double distance = (CoralMeasurements.HeightFromCenterOffGround.minus(RobotMeasurements.EECamHeightOffGround)
                .in(Meters)) / Math.tan(Units.degreesToRadians(RobotMeasurements.EECamForward.in(Radians) + ty));

            double minTimeToObject = distance / Controls.MaxDriveMeterS;
            // kp used to control rotational rate of robot
            double kp = 1 / minTimeToObject;
            drive.setControl(driveBackward.withRotationalRate(Units.degreesToRadians(tx) * kp));
        } else {
            if (hasSeenCoral) {
                System.out.println("lost sight of coral");
                countSinceLastSeenCoral.start();
                hasSeenCoral = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearXYFeedbackOverride();
        PPHolonomicDriveController.clearRotationFeedbackOverride();
        vision.setEndEffectorPipeline(Vision.EndEffectorPipeline.DriverView);
        countSinceLastSeenCoral.stop();
        countSinceLastSeenCoral.reset();
    }

    @Override
    public boolean isFinished() { return endEffector.hasCoral() || countSinceLastSeenCoral.hasElapsed(hasSeenCoralSeconds.get()); }
}
