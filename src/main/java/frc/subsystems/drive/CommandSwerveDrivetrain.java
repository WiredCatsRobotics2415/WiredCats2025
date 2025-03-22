package frc.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.Controls;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.RuntimeConstants;
import frc.constants.Subsystems.DriveConstants;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.constants.TunerConstants;
import frc.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.subsystems.vision.Vision;
import frc.utils.AllianceDependent;
import frc.utils.LimelightHelpers.PoseEstimate;
import frc.utils.driver.DashboardManager;
import frc.utils.math.Statistics;
import frc.utils.simulation.MapleSimSwerveDrivetrain;
import frc.utils.tuning.TuneableNumber;
import frc.utils.tuning.TuneableProfiledPIDController;
import frc.utils.tuning.TuningModeTab;
import java.util.ArrayList;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier simNotifier = null;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);
    public final SwerveRequest.FieldCentric driveOpenLoopFieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Controls.MaxDriveMeterS * 0.05).withRotationalDeadband(Controls.MaxAngularRadS * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    public final SwerveRequest.RobotCentric driveOpenLoopRobotCentricRequest = new SwerveRequest.RobotCentric()
        .withDeadband(Controls.MaxDriveMeterS * 0.05).withRotationalDeadband(Controls.MaxAngularRadS * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.MotionMagicExpo);
    public final SwerveRequest.FieldCentricFacingAngle driveToPositionFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public static final Transform2d redDriveToGoalCompensation = new Transform2d(0, 0, kRedAlliancePerspectiveRotation);

    @Getter private TuneableProfiledPIDController driveToPositionXController = new TuneableProfiledPIDController(
        DriveConstants.XTranslationPID,
        new Constraints(DriveConstants.BaseVelocityMax.get(), DriveConstants.BaseXAccelerationMax.get()), "DriveToX");
    @Getter private TuneableProfiledPIDController driveToPositionYController = new TuneableProfiledPIDController(
        DriveConstants.YTranslationPID,
        new Constraints(DriveConstants.BaseVelocityMax.get(), DriveConstants.BaseYAccelerationMax.get()), "DriveToY");
    private PhoenixPIDController driveToPositionHeadingController = new PhoenixPIDController(DriveConstants.HeadingkP,
        DriveConstants.HeadingkI, DriveConstants.HeadingkD);

    private TuneableNumber resetPoseSamples = new TuneableNumber(40, "Drive/ResetPoseSamples");

    private TuneableNumber singleTagBaseDistrust = new TuneableNumber(0.7, "Drive/singleTagBaseDistrust");
    private TuneableNumber singleTagDistanceFromCurrent = new TuneableNumber(1, "Drive/singleTagDistanceFromCurrent");
    private TuneableNumber singleTagDistanceFromTag = new TuneableNumber(1.3, "Drive/singleTagDistanceFromTag");

    private TuneableNumber headingTolerance = new TuneableNumber(DriveConstants.HeadingTolerance,
        "Drive/HeadingTolerance");

    private Pose2d currentAutoDriveTarget = Pose2d.kZero;

    private VisionPoseFuser poseFuser = new VisionPoseFuser(this);
    private Vision vision = Vision.getInstance();

    public enum PoseEstimationType {
        SingleTag, Global
    }

    private PoseEstimationType currentPoseEstimationType = PoseEstimationType.Global;
    private LimelightsForElements currentSingleTagLLs;
    private int currentSingleTagId;

    private static CommandSwerveDrivetrain instance;

    private CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
            // Start robot out farther in field so collisions don't apply (yet)
            resetPose(new Pose2d(3, 3, Rotation2d.kZero));
        }
        configureAutoBuilder();

        driveToPositionHeadingController.setTolerance(DriveConstants.HeadingTolerance);
        driveToPositionHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        driveToPositionFacingAngleRequest.HeadingController = driveToPositionHeadingController;

        TuningModeTab.getInstance().addCommand("Reset Pose from Limelight",
            resetPoseFromLimelight().ignoringDisable(true));
        TuningModeTab.getInstance().addCommand("Reset Rotation from MT1",
            resetRotationFromLimelightMT1().ignoringDisable(true));

        if (RuntimeConstants.TuningMode) {
            DriveCharacterization.enable(this);

            DriveConstants.PPTranslationP.addListener(newP -> {
                DriveConstants.PPTranslationPID = new PIDConstants(newP);
                DriveConstants.PathFollowingController = new PPHolonomicDriveController(DriveConstants.PPTranslationPID,
                    DriveConstants.RotationPID);
            });
            DriveConstants.RotationP.addListener(newP -> {
                DriveConstants.RotationPID = new PIDConstants(newP);
                DriveConstants.PathFollowingController = new PPHolonomicDriveController(DriveConstants.PPTranslationPID,
                    DriveConstants.RotationPID);
            });

            headingTolerance.addListener((newTol) -> {
                driveToPositionHeadingController.setTolerance(newTol);
            });

            // Add torque safety to all motors
            // Only uncomment this if you are risking grinding a gear, otherwise make sure
            // all swerve requests are Slew Rate Limited to prevent gear wear
            /*
             * SwerveRequest coastAllSwerve = new SwerveRequest() { boolean sentMotorConfigs = false;
             *
             * @Override public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) { for (SwerveModule<?, ?, ?> m : modulesToApply) { if (!sentMotorConfigs) ((TalonFX) m.getDriveMotor()).setNeutralMode(NeutralModeValue.Coast); ((TalonFX) m.getDriveMotor()).set(0); if (!sentMotorConfigs) ((TalonFX) m.getSteerMotor()).setNeutralMode(NeutralModeValue.Coast); ((TalonFX) m.getSteerMotor()).set(0); } sentMotorConfigs = true; return StatusCode.OK; } }; String[] names = new String[] { "Front Left", "Front Right", "Back Left", "Back Right" }; int i = 0; for (SwerveModule<TalonFX, TalonFX, CANcoder> m : getModules()) { TorqueSafety.getInstance().addMotor(m.getDriveMotor().getSupplyCurrent().asSupplier(), applyRequest(() -> coastAllSwerve).withName(names[i] + " Drive")); TorqueSafety.getInstance().addMotor(m.getSteerMotor().getSupplyCurrent().asSupplier(), applyRequest(() -> coastAllSwerve).withName(names[i] + " Steer")); i++; }
             */
        }

        DashboardManager.getInstance().addBoolSupplier(true, "Using ST",
            () -> currentPoseEstimationType == PoseEstimationType.SingleTag);
    }

    public static CommandSwerveDrivetrain getInstance() {
        if (instance == null) instance = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        return instance;
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotMeasurements.PPRobotConfig;
            AutoBuilder.configure(() -> getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds,
                    feedforwards) -> setControl(autoRequest.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                DriveConstants.PathFollowingController, RobotConfig.fromGUISettings(),
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                ex.getStackTrace());
            new Alert("Failed to load PathPlanner config and configure AutoBuilder", AlertType.kError).set(true);
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

        SwerveDriveState currentState = getState();
        poseFuser.sendLimelightsOrientation(currentState);
        if (currentPoseEstimationType == PoseEstimationType.SingleTag) {
            PoseEstimate singleTag = vision.getSingleTagPoseEstimate(currentSingleTagLLs, currentSingleTagId);
            if (singleTag != null) {
                double distrust = singleTagBaseDistrust.get() +
                    singleTagDistanceFromCurrent.get() *
                        currentState.Pose.getTranslation().getDistance(singleTag.pose.getTranslation()) +
                    singleTagDistanceFromTag.get() * singleTag.avgTagDist;
                System.out.println(
                    "Fusing singletag measurement w/ distrust: " + distrust + " and ts: " + singleTag.timestampSeconds);
                addVisionMeasurement(singleTag.pose, Utils.fpgaToCurrentTime(singleTag.timestampSeconds),
                    VecBuilder.fill(distrust, distrust, 999999));
            }
        } else {
            poseFuser.update(currentState);
        }

        Logger.recordOutput("Drive/Pose", currentState.Pose);
        if (mapleSimSwerveDrivetrain != null) Logger.recordOutput("Drive/SimulationPose",
            mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
        Logger.recordOutput("Drive/ModuleStates", currentState.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", currentState.ModuleTargets);
        Logger.recordOutput("Drive/AutoDrive", currentAutoDriveTarget);
    }

    /**
     * Sets the pose estimator to fuse only this tag seen by this limelights until you interrupt this command. You MUST interrupt this command, otherwise the pose estimator will stay this way until you the switchPoseEstimator command is run.
     */
    public Command focusOnTagWhenSeenTemporarily(LimelightsForElements limelights, int tag) {
        return new WaitUntilCommand(() -> vision.nearestTagToAtLeastOneOf(limelights) == tag)
            .andThen(Commands.run(() ->
            {
                currentPoseEstimationType = PoseEstimationType.SingleTag;
                currentSingleTagLLs = limelights;
                currentSingleTagId = tag;
            })).finallyDo(() -> {
                currentPoseEstimationType = PoseEstimationType.Global;
            });
    }

    public Command focusOnTagWhenSeenTemporarily(LimelightsForElements limelights, AllianceDependent<int[]> tagSet) {
        return new WaitUntilCommand(() -> vision.aLimelightCanSeeOneOf(limelights, tagSet.get()))
            .andThen(Commands.run(() ->
            {
                currentPoseEstimationType = PoseEstimationType.SingleTag;
                currentSingleTagLLs = limelights;
                currentSingleTagId = vision.nearestTagToAtLeastOneOf(limelights);
            })).finallyDo(() -> {
                currentPoseEstimationType = PoseEstimationType.Global;
            });
    }

    public Command switchPoseEstimator(PoseEstimationType poseEstimationType) {
        return Commands.runOnce(() -> this.currentPoseEstimationType = poseEstimationType);
    }

    /**
     * Uses Pathplanner's pathfinding (obstactle avoiding trajectory generation) to drive to a pose w/ same constants and smoothing as autonomous
     *
     * @param goalPose The pose to drive to
     */
    public Command pathfindTo(Pose2d goalPose) {
        try {
            return new PathfindingCommand(goalPose, DriveConstants.DefaultPathConstraints, () -> getState().Pose,
                () -> getState().Speeds,
                (speeds,
                    feedforwards) -> setControl(autoRequest.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                DriveConstants.PathFollowingController, RobotMeasurements.PPRobotConfig, this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                ex.getStackTrace());
            return null;
        }
    }

    /**
     * Uses simple PID control to drive the robot to a position, with different translation PID constants than Pathplanner. and using Phoenix's FieldCentricFacingAngle SwerveRequest
     *
     * @param goalPose                   The pose to drive to
     * @param translationToleranceMeters The tolerance allowed for the translation controllers
     */
    public Command driveTo(Pose2d goalPose, double translationToleranceMeters) {
        driveToPositionXController.setTolerance(translationToleranceMeters);
        driveToPositionYController.setTolerance(translationToleranceMeters);

        double maxSpeedMS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        return runOnce(() -> {
            currentAutoDriveTarget = goalPose;
            SwerveDriveState currentState = getState();
            Pose2d currentPose = currentState.Pose;
            ChassisSpeeds speeds = currentState.Speeds;
            driveToPositionXController.reset(new State(currentPose.getX(), speeds.vxMetersPerSecond));
            driveToPositionYController.reset(new State(currentPose.getY(), speeds.vyMetersPerSecond));
            driveToPositionHeadingController.reset();
        }).andThen(applyRequest(() -> {
            boolean isBlue = AllianceDependent.isCurrentlyBlue();

            Pose2d currentPose = getState().Pose;
            Pose2d allianceCompensatedGoal = isBlue ? goalPose : goalPose.plus(redDriveToGoalCompensation);
            double velocityX = MathUtil.clamp(
                driveToPositionXController.calculate(currentPose.getX(), allianceCompensatedGoal.getX()), -maxSpeedMS,
                maxSpeedMS);
            double velocityY = MathUtil.clamp(
                driveToPositionYController.calculate(currentPose.getY(), allianceCompensatedGoal.getY()), -maxSpeedMS,
                maxSpeedMS);
            double allianceCompensation = isBlue ? 1 : -1;

            return driveToPositionFacingAngleRequest.withTargetDirection(allianceCompensatedGoal.getRotation())
                .withVelocityX(velocityX * allianceCompensation).withVelocityY(velocityY * allianceCompensation);
        })).until(() -> {
            // System.out.println("xCon: " + driveToPositionXController.getLastCalculation());
            // System.out.println("yCon: " + driveToPositionYController.getLastCalculation());
            // return (driveToPositionXController.atGoal() && driveToPositionYController.atGoal()
            // && driveToPositionHeadingController.atSetpoint())
            // || (Math.abs(driveToPositionXController.getLastCalculation()) < Controls.MaxDriveMeterS * 0.025
            // && Math.abs(driveToPositionYController.getLastCalculation()) < Controls.MaxDriveMeterS * 0.025);
            return (driveToPositionXController.atGoal() && driveToPositionYController.atGoal()
                && driveToPositionHeadingController.atSetpoint());
        }).finallyDo(() -> {
            currentAutoDriveTarget = null;
        });
    }

    public double maxTimeToGetToPose(Pose2d goal) {
        SwerveDriveState currentState = getState();
        Pose2d currentPose = currentState.Pose;

        double xTime = driveToPositionXController.timeToGetTo(currentPose.getX(), goal.getX());
        double yTime = driveToPositionYController.timeToGetTo(currentPose.getY(), goal.getY());
        return Math.max(xTime, yTime);
    }

    public Command resetPoseFromLimelight() {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        return run(() -> {
            Pose2d currentAvg = vision.getCurrentAveragePose();
            if (currentAvg != null) poses.add(currentAvg);
        }).until(() -> poses.size() == (int) resetPoseSamples.get()).andThen(runOnce(() -> {
            double sumX = 0.0d, sumY = 0.0d;
            for (Pose2d pose : poses) {
                sumX += pose.getX();
                sumY += pose.getY();
            }
            resetPose(new Pose2d(sumX / poses.size(), sumY / poses.size(), getState().Pose.getRotation()));
            poses.clear();
        }));
    }

    public Command resetRotationFromLimelightMT1() {
        ArrayList<Rotation2d> rotations = new ArrayList<Rotation2d>();

        return run(() -> {
            Rotation2d avgRotation = vision.getCurrentAverageRotation();
            if (avgRotation != null) rotations.add(avgRotation);
        }).until(() -> rotations.size() == (int) resetPoseSamples.get()).andThen(runOnce(() -> {
            resetRotation(Statistics.circularMeanRemoveOutliers(rotations, rotations.size()));
            rotations.clear();
        }));
    }

    @Getter private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(Seconds.of(kSimLoopPeriod),
            RobotMeasurements.RobotWeight, RobotMeasurements.BumperToBumper, RobotMeasurements.BumperToBumper,
            DCMotor.getKrakenX60Foc(1), DCMotor.getFalcon500(1), RobotMeasurements.SwerveModuleConfig.wheelCOF,
            getModuleLocations(), getPigeon2(), getModules(), TunerConstants.FrontLeft, TunerConstants.FrontRight,
            TunerConstants.BackLeft, TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.1); // wait for simulation to update
        }
        super.resetPose(pose);
    }
}
