
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

public class AutoIntake extends Command {
    private Vision vision;
    private EndEffector endEffector;
    private CommandSwerveDrivetrain drive;

    private TuneableNumber hasSeenCoralSeconds;

    private SwerveRequest.RobotCentric driveBackward;

    private boolean hasSeenCoral;
    public Timer countSinceLastSeenCoral;

    @Override
    public void initialize() {
        hasSeenCoral = false;
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

            double distanceV = (RobotMeasurements.EECamHeightOffGround
                .minus(CoralMeasurements.HeightFromCenterOffGround).in(Meters)) /
                Math.tan(Units.degreesToRadians(RobotMeasurements.EECamForward.in(Radians) + ty));
            double distanceH = Math.tan(Units.degreesToRadians(tx)) * distanceV;
            System.out.println("horizontal" + distanceH);
            double distance = Math.sqrt((distanceV * distanceV) + (distanceH * distanceH));
            System.out.println(distance);

            double minTimeToObject = distance / Controls.MaxDriveMeterS;
            // kp used to control rotational rate of robot
            // drive.setControl(driveBackward.withRotationalRate(Units.degreesToRadians(tx) / minTimeToObject));
            drive.setControl(driveBackward.withVelocityY(-distanceH / minTimeToObject)
                .withRotationalRate(Units.degreesToRadians(tx)));
            System.out.println(Units.degreesToRadians(tx));
        } else {
            if (hasSeenCoral) {
                vision.setEndEffectorPipeline(Vision.EndEffectorPipeline.DriverView);
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
        System.out.println("ended" + interrupted);
    }

    @Override
    public boolean isFinished() {
        return endEffector.hasCoral() || countSinceLastSeenCoral.hasElapsed(hasSeenCoralSeconds.get());
    }
}
