package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.commands.Dealgae;
import frc.commands.GenericAutomation;
import frc.commands.GenericAutomation.AutomationMode;
import frc.commands.IntakeFromHPS;
import frc.commands.ScoreCoral;
import frc.commands.ScoreCoral.Level;
import frc.commands.ScoreCoral.Side;
import frc.constants.Controls;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Subsystems.DriveConstants;
import frc.constants.Subsystems.LEDStripConstants.UseableColor;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.drive.CommandSwerveDrivetrain.PoseEstimationType;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.leds.LEDStrip;
import frc.subsystems.superstructure.SuperStructure;
import frc.subsystems.vision.Vision;
import frc.utils.driver.DashboardManager;
import frc.utils.math.AdjustableSLR;
import frc.utils.tuning.TuneableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private static RobotContainer instance;
    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private SuperStructure superstructure = SuperStructure.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    private LEDStrip ledStrip = LEDStrip.getInstance();
    private @Getter OI oi = OI.getInstance();
    private Vision vision = Vision.getInstance();

    private LoggedDashboardChooser<Command> autoChooser;

    public enum TeleopDriveMode {
        Normal, MinorAdjustment
    }

    @Getter private TeleopDriveMode currentTeleopDriveMode = TeleopDriveMode.Normal;

    private AdjustableSLR driveXLimiter = new AdjustableSLR(DriveConstants.BaseXAccelerationMax.get());
    private AdjustableSLR driveYLimiter = new AdjustableSLR(DriveConstants.BaseYAccelerationMax.get());
    private AdjustableSLR driveRotationLimiter = new AdjustableSLR(DriveConstants.BaseRotationAccelMax.get());
    private TuneableNumber minorAdjXPct = new TuneableNumber(0.2, "Drive/minorAdjXPct");
    private TuneableNumber minorAdjYPct = new TuneableNumber(0.2, "Drive/minorAdjYPct");

    private RobotContainer() {
        setupAuto();
        configureControls();
        configureTriggers();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void setupAuto() {
        // Put Auto named commands here
        NamedCommands.registerCommand("L4", new ScoreCoral(Side.Left, Level.L4));
        NamedCommands.registerCommand("L3", new ScoreCoral(Side.Left, Level.L3));
        NamedCommands.registerCommand("L2", new ScoreCoral(Side.Left, Level.L2));
        NamedCommands.registerCommand("L1", new ScoreCoral(Side.Left, Level.L1));
        // these are not right... i need to see how exactly we need to move subsystems to do ground and source intake
        NamedCommands.registerCommand("GroundIntake",
            superstructure.beThereAsap(Presets.GroundIntake).alongWith(endEffector.intakeAndWaitForCoral()));
        NamedCommands.registerCommand("SourceIntake",
            superstructure.beThereAsap(Presets.IntakeFromHPS).alongWith(endEffector.intakeAndWaitForCoral()));
        NamedCommands.registerCommand("FocusPEOnReef",
            drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, ReefMeasurements.reefIds));
        NamedCommands.registerCommand("SwitchToGlobalPE",
            new InstantCommand(() -> drive.switchPoseEstimator(PoseEstimationType.Global)));
        // NamedCommands.registerCommand("align", vision.singleTagTrack());

        autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser(""));

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Pathplanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Pathplanner/TargetPose", targetPose);
        });
        PathPlannerLogging.setLogCurrentPoseCallback((currentPose) -> {
            Logger.recordOutput("Pathplanner/CurrentPose", currentPose);
        });
    }

    private void configureControls() {
        oi.binds.get(OI.Bind.SeedFieldCentric).onTrue(drive.resetRotationFromLimelightMT1().ignoringDisable(true));

        drive.setDefaultCommand(drive.applyRequest(() -> {
            if (currentTeleopDriveMode == TeleopDriveMode.MinorAdjustment) {
                double[] input = oi.getRawXY();
                double x = input[1], y = input[0];
                // this mode disables rotation, and sets the maximum speed of the robot to the minorAdjPct speed.
                x = x / (1 / minorAdjXPct.get());
                y = y / (1 / minorAdjYPct.get());
                // x = Math.abs(x) > minorAdjXPct.get() ? Math.signum(x) * minorAdjXPct.get() : 0;
                // y = Math.abs(y) > minorAdjYPct.get() ? Math.signum(y) * minorAdjYPct.get() : 0;
                return drive.driveOpenLoopRobotCentricRequest
                    .withVelocityX(driveXLimiter.calculate(-x * Controls.MaxDriveMeterS))
                    .withVelocityY(driveYLimiter.calculate(-y * Controls.MaxDriveMeterS))
                    .withRotationalRate(driveRotationLimiter.calculate(0) * Controls.MaxAngularRadS);
            }
            double[] linearInput = oi.getXY();
            double x = linearInput[1], y = linearInput[0];
            double rotation = oi.getRotation();
            return drive.driveOpenLoopFieldCentricRequest
                .withVelocityX(driveXLimiter.calculate(-x * Controls.MaxDriveMeterS))
                .withVelocityY(driveYLimiter.calculate(-y * Controls.MaxDriveMeterS))
                .withRotationalRate(driveRotationLimiter.calculate(-rotation) * Controls.MaxAngularRadS);
        }).withName("Teleop Default"));
        oi.binds.get(OI.Bind.ChangeTeleopMode).debounce(0.25, DebounceType.kRising)
            .onTrue(Commands.runOnce(() -> currentTeleopDriveMode = TeleopDriveMode.MinorAdjustment))
            .onFalse(Commands.runOnce(() -> currentTeleopDriveMode = TeleopDriveMode.Normal));

        oi.binds.get(OI.Bind.ManualElevatorUp)
            .whileTrue(Commands.run(() -> superstructure.changeElevatorGoalSafely(Inches.of(1.0))));
        oi.binds.get(OI.Bind.ManualElevatorDown)
            .whileTrue(Commands.run(() -> superstructure.changeElevatorGoalSafely(Inches.of(-1.0))));

        oi.binds.get(OI.Bind.ManualArmForward)
            .whileTrue(Commands.run(() -> superstructure.changeArmGoalSafely(Degrees.of(-1.0))));
        oi.binds.get(OI.Bind.ManualArmBack)
            .whileTrue(Commands.run(() -> superstructure.changeArmGoalSafely(Degrees.of(1.0))));

        oi.binds.get(OI.Bind.Shoot).onTrue(endEffector.toggleOuttake()).onFalse(endEffector.turnOff());
        oi.binds.get(OI.Bind.DeAlgae).onTrue(endEffector.toggleIntakeAlgae());
        oi.binds.get(OI.Bind.ManualIntake).onTrue(endEffector.toggleIntakeCoral());
        // In case it should be A intakes coral, outtakes algae and B vice versa...
        // Dealgae is button A:
        // oi.binds.get(OI.Bind.DeAlgae).onTrue(endEffector.toggleIntakeCoral().alongWith(coralIntake.toggleIntake()))
        // .onFalse(endEffector.turnOff().alongWith(coralIntake.turnOffRollers()));
        // Shoot is button B:
        // oi.binds.get(OI.Bind.DeAlgae).onTrue(endEffector.toggleOuttake().alongWith(coralIntake.toggleOuttake())).onFalse(endEffector.turnOff().alongWith(coralIntake.turnOffRollers()));

        oi.binds.get(OI.Bind.StowPreset).onTrue(CommonCommands.stowFromGroundIntake().ignoringDisable(true));
        oi.binds.get(OI.Bind.IntakeFromGround)
            .onTrue(superstructure.beThereAsap(Presets.GroundIntake).andThen(endEffector.intakeAndWaitForCoral()));
        oi.binds.get(OI.Bind.IntakeFromHPS).onTrue(new IntakeFromHPS());
        oi.binds.get(OI.Bind.DealgaePreset).onTrue(new Dealgae());
        oi.binds.get(OI.Bind.AutoScoreLeftL1).onTrue(new ScoreCoral(Side.Left, Level.L1));
        oi.binds.get(OI.Bind.AutoScoreLeftL2).onTrue(new ScoreCoral(Side.Left, Level.L2));
        oi.binds.get(OI.Bind.AutoScoreLeftL3).onTrue(new ScoreCoral(Side.Left, Level.L3));
        oi.binds.get(OI.Bind.AutoScoreLeftL4).onTrue(new ScoreCoral(Side.Left, Level.L4));
        oi.binds.get(OI.Bind.AutoScoreRightL1).onTrue(new ScoreCoral(Side.Right, Level.L1));
        oi.binds.get(OI.Bind.AutoScoreRightL2).onTrue(new ScoreCoral(Side.Right, Level.L2));
        oi.binds.get(OI.Bind.AutoScoreRightL3).onTrue(new ScoreCoral(Side.Right, Level.L3));
        oi.binds.get(OI.Bind.AutoScoreRightL4).onTrue(new ScoreCoral(Side.Right, Level.L4));

        DashboardManager.getInstance().addBoolSupplier(true, "Auto drive",
            () -> ScoreCoral.getCurrentAutomationMode().equals(AutomationMode.PresetAndAlign), null);
        oi.binds.get(OI.Bind.ToggleScorePresetsAlignDrive).onTrue(new InstantCommand(() -> {
            if (GenericAutomation.getCurrentAutomationMode().equals(AutomationMode.PresetAndAlign)) {
                GenericAutomation.setCurrentAutomationMode(AutomationMode.PresetOnly);
            } else {
                GenericAutomation.setCurrentAutomationMode(AutomationMode.PresetAndAlign);
            }
        }));

        // oi.binds.get(OI.Bind.AutoIntakeFromGround).whileTrue(
        // superstructure.beThereAsap(Presets.GroundIntake).andThen(Commands.waitUntil(superstructure::allAtGoal))
        // .andThen(new AutoIntake().withTimeout(5)).andThen(CommonCommands.stowFromGroundIntake()));

        oi.binds.get(OI.Bind.ProcessorPreset).onTrue(
            superstructure.beThereAsap(Presets.ProcessorScore).andThen(Commands.waitUntil(superstructure::allAtGoal)));
    }

    private void configureTriggers() {
        // Triggers that interact across multiple subsystems/utils should be defined here
        // Flip driver camera based on arm angle
        new Trigger(() -> {
            return arm.getMeasurement().in(Degrees) > 90.0d;
        }).onTrue(new InstantCommand(() -> {
            vision.setEndEffectorStreamOrientation(true);
        }).ignoringDisable(true)).onFalse(new InstantCommand(() -> {
            vision.setEndEffectorStreamOrientation(false);
        }).ignoringDisable(true));

        // Flash the leds when a coral is scored
        // new Trigger(() -> {
        // return (endEffector.isOuttakingAlgae() && !endEffector.cameraTrigger())
        // || (endEffector.isOuttakingCoral() && !endEffector.irSensorTrigger());
        // }).onTrue(new WaitCommand(0.5)
        // .andThen(ledStrip.flash(UseableColor.SkyBlue, Seconds.of(0.3), Seconds.of(0.3)).withTimeout(1.2))
        // .andThen(RobotStatus.setRobotStateOnce(RobotState.Enabled)));

        new Trigger(endEffector::hasAlgae)
            .onTrue(ledStrip.flash(UseableColor.SkyBlue, Seconds.of(0.3), Seconds.of(0.3)));
        new Trigger(endEffector::hasCoral)
            .onTrue(ledStrip.flash(UseableColor.SkyBlue, Seconds.of(0.3), Seconds.of(0.3)));
        // new Trigger(endEffector::hasCoral).onTrue(coralIntake.turnOffRollers());
    }

    public void periodic() {
        double[] ssLimits = superstructure.recommendedDriveAccelLimits();
        driveXLimiter.setRateLimit(ssLimits[0]);
        driveYLimiter.setRateLimit(ssLimits[1]);
        driveRotationLimiter.setRateLimit(ssLimits[2]);

        drive.getDriveToPositionXController()
            .setConstraints(new Constraints(DriveConstants.BaseVelocityMax.get(), ssLimits[0]));
        drive.getDriveToPositionYController()
            .setConstraints(new Constraints(DriveConstants.BaseVelocityMax.get(), ssLimits[1]));
    }

    public Command getAutonomousCommand() { return autoChooser.get(); }
}
