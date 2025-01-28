package frc.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.constants.Controls;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.DriveAutoConstants;
import frc.constants.Subsystems.VisionConstants;
import frc.constants.TunerConstants;
import frc.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.subsystems.vision.Vision;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.tuning.TuningModeTab;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.Position);
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Controls.MaxDriveMeterS * 0.05).withRotationalDeadband(Controls.MaxAngularRadS * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.Position)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    private Vision vision = Vision.getInstance();
    private static CommandSwerveDrivetrain instance;

    private CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        if (RuntimeConstants.TuningMode) {
            DriveCharacterization.enable(this);
            TuningModeTab.getInstance().addCommand("Reset Pose from Limelight",
                resetPoseFromLimelight().ignoringDisable(true));
        }
    }

    public static CommandSwerveDrivetrain getInstance() {
        if (instance == null) instance = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        return instance;
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(() -> getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds,
                    feedforwards) -> setControl(autoRequest.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                DriveAutoConstants.PathFollowingController, config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                ex.getStackTrace());
            new Alert("Failed to load PathPlanner config and configure AutoBuilder", AlertType.kError);
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective. If we haven't applied the operator perspective before, then we should apply it regardless of DS state. This allows us to correct the perspective in case the robot code restarts mid-match. Otherwise, only check and apply the operator perspective if the DS is disabled. This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        updateLimelights();

        SwerveDriveState currentState = getState();
        Logger.recordOutput("Drive/Pose", currentState.Pose);
        Logger.recordOutput("Drive/ModuleStates", currentState.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", currentState.ModuleTargets);
    }

    public Command pathfindTo(Pose2d goalPose) {
        try {
            return new PathfindingCommand(goalPose, DriveAutoConstants.DefaultPathConstraints, () -> getState().Pose,
                () -> getState().Speeds,
                (speeds,
                    feedforwards) -> setControl(autoRequest.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                DriveAutoConstants.PathFollowingController, RobotConfig.fromGUISettings(), this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                ex.getStackTrace());
            return null;
        }
    }

    public void updateLimelights() {
        vision.sendOrientation(this.getState().Pose.getRotation());
        PoseEstimate[] estimates = vision.getPoseEstimates();

        for (int i = 0; i < estimates.length; i++) {
            PoseEstimate estimate = estimates[i];
            if (Math.abs((Units.radiansToRotations(this.getState().Speeds.omegaRadiansPerSecond))) < 720
                && estimate.tagCount > 0) {
                setVisionMeasurementStdDevs(VisionConstants.megatag2StdDev);
                addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
            }
        }
    }

    private Command resetPoseFromLimelight() {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        return run(() -> poses.add(vision.getCurrentAveragePose())).until(() -> poses.size() == 20)
            .andThen(runOnce(() ->
            {
                double sumX = 0.0d, sumY = 0.0d;
                for (Pose2d pose : poses) {
                    sumX += pose.getX();
                    sumY += pose.getY();
                }
                resetPose(new Pose2d(sumX / poses.size(), sumY / poses.size(), getState().Pose.getRotation()));
                poses.clear();
            }));
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
