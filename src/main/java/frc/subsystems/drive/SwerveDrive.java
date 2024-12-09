package frc.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.constants.Controls;
import frc.constants.Subsystems.SwerveConstants;
import frc.constants.Subsystems.VisionConstants;
import frc.constants.TunerConstants;
import frc.subsystems.vision.Vision;
import frc.util.LimelightHelpers.PoseEstimate;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can
 * be used in command-based projects easily.
 */
public class SwerveDrive extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Vision vision;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity);
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Controls.MaxDriveMeterS * 0.05)
        .withRotationalDeadband(Controls.MaxAngularRadS * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(Controls.MaxDriveMeterS * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private SwerveDriveInputsAutoLogged inputs = new SwerveDriveInputsAutoLogged();

    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants,
        double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        initialization();
    }

    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants,
        SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        initialization();
    }

    private void initialization() {
        addExtraMotorConfigs();
        configurePathPlanner();
        driveFacingAngle.HeadingController = SwerveConstants.HeadingPIDController;
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.registerTelemetry((SwerveDrivetrain.SwerveDriveState state) -> {
            inputs.odometryPose = state.Pose;
            inputs.goalStates = state.ModuleTargets;
            inputs.actualStates = state.ModuleStates;

            Logger.processInputs("SwerveDrive", inputs);
        });

        vision = Vision.getInstance();
    }

    private void addExtraMotorConfigs() {
        for (SwerveModule m : this.Modules) {
            m.getDriveMotor().getConfigurator()
                .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.2));
        }
    }

    /**
     * runs configureHolonomic. Must only be run once after all named commands are defined.
     */
    private void configurePathPlanner() {
        double driveBaseRadius = m_moduleLocations[0].getNorm();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(() -> this.getState().Pose, // Supplier of current
            // robot pose
            this::seedFieldRelative, // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of
            // ChassisSpeeds to
            // drive the robot
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
                // likely
                // live in your Constants class
                new PIDConstants(0.1, 0.0, 0), // Translation PID constants
                new PIDConstants(0.1, 0.0, 0), // Rotation PID constants
                3, // Max module speed, in m/s
                driveBaseRadius, // Drive base radius in meters. Distance from robot center
                // to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for
            // the options here
            ), () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, this // Reference to this subsystem to set requirements
        );
    }

    /**
     * @return a Command that takes a SwerveRequest supplier and applies it for as long as
     *         this command runs.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command faceAngleWithTolerance(Rotation2d angle) {
        Command command = new Command() {
            public void execute() {
                TunerConstants.DriveTrain.setControl(
                    TunerConstants.DriveTrain.driveFacingAngle.withTargetDirection(angle));
            };

            @Override
            public boolean isFinished() {
                double rotation = TunerConstants.DriveTrain.m_odometry
                    .getEstimatedPosition().getRotation().getDegrees();
                return MathUtil.isNear(angle.getDegrees(), rotation,
                    SwerveConstants.HeadingControllerTolerance);
            }
        };
        command.addRequirements(this);
        return command;
    }

    /**
     * @return Get the current robot-centric chassis speeds, directly from the module's
     *         actual states.
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void reset() {
        for (int i = 0; i < 4; i++) {
            getModule(i).apply(new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                DriveRequestType.OpenLoopVoltage);
        }
    }

    @Override
    public void periodic() {
        // Straight from
        // https://www.chiefdelphi.com/t/introducing-megatag2-by-limelight-vision/461243
        vision.sendOrientation(this.m_pigeon2.getRotation2d());
        PoseEstimate poseEstimate = vision.getShooterResults();
        if (Math.abs(this.m_pigeon2.getRate()) < 480 && poseEstimate.tagCount > 0) {
            this.m_odometry.setVisionMeasurementStdDevs(VisionConstants.stdDevs);
            this.m_odometry.addVisionMeasurement(poseEstimate.pose,
                poseEstimate.timestampSeconds);
        }
    }
}
