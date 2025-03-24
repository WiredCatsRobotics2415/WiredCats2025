
package frc.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.constants.Subsystems.DriveConstants;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.coralintake.CoralIntake;
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

    private static TuneableNumber hasSeenCoralSeconds = new TuneableNumber(0.5, "AutoIntaking/hasSeenCoralSeconds");

    private SwerveRequest.RobotCentric driveBackward;

    private boolean hasSeenCoral;
    public Timer countSinceLastSeenCoral;
    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    @Override
    public void initialize() {
        System.out.println("autointaking!");
        hasSeenCoral = false;
        driveHeading.HeadingController = new PhoenixPIDController(DriveConstants.HeadingkP, 0,
            DriveConstants.HeadingkD);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        countSinceLastSeenCoral = new Timer();
        vision = Vision.getInstance();
        endEffector = EndEffector.getInstance();
        drive = CommandSwerveDrivetrain.getInstance();
        addRequirements(drive);
        driveBackward = new SwerveRequest.RobotCentric().withVelocityX(-0.4 * Controls.MaxDriveMeterS); // back of robot = intaking side
        RobotStatus.setRobotStateOnce(RobotState.AutoGroundIntaking);
        endEffector.intakeAndWaitForCoral().schedule();
        CoralIntake.getInstance().toggleIntake().schedule();
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

            System.out.println(tx);
            if (tx > tolerance || tx < -tolerance) {
                System.out.println("tx outside of tolerance");
                drive.setControl(driveHeading
                    .withTargetDirection(drive.getState().Pose.getRotation().minus(Rotation2d.fromDegrees(-tx))));
            } else {
                System.out.println("driving backward");
                drive.setControl(driveBackward);
            }
        } else {
            if (hasSeenCoral) {
                System.out.println("lost sight of coral");
                countSinceLastSeenCoral.start();
                hasSeenCoral = false;
            } else {
                System.out.println("has not seen coral");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        countSinceLastSeenCoral.reset();
        countSinceLastSeenCoral.stop();
        vision.setEndEffectorPipeline(EndEffectorPipeline.DriverView);
        CoralIntake.getInstance().turnOffRollers();
        System.out.println("ended" + interrupted);
    }

    @Override
    public boolean isFinished() {
        return endEffector.hasCoral() || countSinceLastSeenCoral.hasElapsed(hasSeenCoralSeconds.get());
    }
}
