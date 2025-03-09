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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.commands.AutoIntake;
import frc.commands.Dealgae;
import frc.commands.Dealgae.DealgaeAutomationMode;
import frc.commands.ScoreCoral;
import frc.commands.ScoreCoral.CoralAutomationMode;
import frc.commands.ScoreCoral.Level;
import frc.commands.ScoreCoral.Side;
import frc.constants.Controls;
import frc.constants.Controls.Presets;
import frc.constants.Subsystems.DriveConstants;
import frc.constants.Subsystems.LEDStripConstants.UseableColor;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.arm.Arm;
import frc.subsystems.coralintake.CoralIntake;
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
    private CoralIntake coralIntake = CoralIntake.getInstance();
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
            superstructure.beThereAsap(Presets.IntakeFromCPS).alongWith(endEffector.intakeAndWaitForCoral()));
        NamedCommands.registerCommand("LockOntoReef", drive.switchToSingleTagWhenAvailable());
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
            double[] linearInput = oi.getXY();
            double x = linearInput[1], y = linearInput[0];
            double rotation = oi.getRotation();
            if (currentTeleopDriveMode == TeleopDriveMode.MinorAdjustment) {
                rotation = 0;
                x = Math.abs(x) > minorAdjXPct.get() ? Math.signum(x) * minorAdjXPct.get() : 0;
                y = Math.abs(y) > minorAdjXPct.get() ? Math.signum(y) * minorAdjYPct.get() : 0;
            }
            return drive.driveOpenLoopRequest.withVelocityX(driveXLimiter.calculate(-x) * Controls.MaxDriveMeterS)
                .withVelocityY(driveYLimiter.calculate(-y) * Controls.MaxDriveMeterS)
                .withRotationalRate(driveRotationLimiter.calculate(-rotation) * Controls.MaxAngularRadS);
        }).withName("Teleop Default"));
        oi.binds.get(OI.Bind.ChangeTeleopMode).debounce(0.5, DebounceType.kRising)
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

        oi.binds.get(OI.Bind.ToggleDealgae).onTrue(endEffector.toggleIntakeAlgae());
        oi.binds.get(OI.Bind.ToggleIntake).onTrue(endEffector.toggleIntakeCoral());
        oi.binds.get(OI.Bind.ToggleOuttake).onTrue(endEffector.toggleOuttake());

        oi.binds.get(OI.Bind.AutoScoreLeftL1).onTrue(new ScoreCoral(Side.Left, Level.L1));
        oi.binds.get(OI.Bind.AutoScoreLeftL2).onTrue(new ScoreCoral(Side.Left, Level.L2));
        oi.binds.get(OI.Bind.AutoScoreLeftL3).onTrue(new ScoreCoral(Side.Left, Level.L3));
        oi.binds.get(OI.Bind.AutoScoreLeftL4).onTrue(new ScoreCoral(Side.Left, Level.L4));
        oi.binds.get(OI.Bind.AutoScoreRightL1).onTrue(new ScoreCoral(Side.Right, Level.L1));
        oi.binds.get(OI.Bind.AutoScoreRightL2).onTrue(new ScoreCoral(Side.Right, Level.L2));
        oi.binds.get(OI.Bind.AutoScoreRightL3).onTrue(new ScoreCoral(Side.Right, Level.L3));
        oi.binds.get(OI.Bind.AutoScoreRightL4).onTrue(new ScoreCoral(Side.Right, Level.L4));
        oi.binds.get(OI.Bind.DealgaePreset).onTrue(new Dealgae());
        oi.binds.get(OI.Bind.IntakeFromGround).onTrue(superstructure.beThereAsap(Presets.GroundIntake));
        oi.binds.get(OI.Bind.IntakeFromHPS).onTrue(superstructure.beThereAsap(Presets.IntakeFromCPS));

        DashboardManager.getInstance().addBoolSupplier(true, "Auto drive",
            () -> ScoreCoral.getCurrentAutomationMode().equals(CoralAutomationMode.PresetAndAlign), null);
        oi.binds.get(OI.Bind.ToggleScorePresetsAlignDrive).onTrue(new InstantCommand(() -> {
            if (ScoreCoral.getCurrentAutomationMode().equals(CoralAutomationMode.PresetAndAlign)) {
                ScoreCoral.setCurrentAutomationMode(CoralAutomationMode.PresetOnly);
            } else {
                ScoreCoral.setCurrentAutomationMode(CoralAutomationMode.PresetAndAlign);
            }

            if (Dealgae.getCurrentAutomationMode().equals(DealgaeAutomationMode.PresetAndAlign)) {
                Dealgae.setCurrentAutomationMode(DealgaeAutomationMode.PresetOnly);
            } else {
                Dealgae.setCurrentAutomationMode(DealgaeAutomationMode.PresetAndAlign);
            }
        }));

        oi.binds.get(OI.Bind.AutoIntakeFromGround).onTrue(new AutoIntake());

        oi.binds.get(OI.Bind.StowPreset).onTrue(superstructure.beThereAsap(Presets.Stow));
    }

    private void configureTriggers() {
        // Triggers that interact across multiple subsystems/utils should be defined here
        // Flip driver camera based on arm angle
        new Trigger(() -> {
            return arm.getMeasurement().in(Degrees) > 90.0d;
        }).onTrue(new InstantCommand(() -> {
            vision.setEndEffectorStreamOrientation(true);
        })).onFalse(new InstantCommand(() -> {
            vision.setEndEffectorStreamOrientation(false);
        }));

        // Flash the leds when a coral is scored
        new Trigger(() -> {
            return endEffector.isOuttaking() && !endEffector.irSensorTrigger();
        }).onTrue(new WaitCommand(0.5)
            .andThen(ledStrip.flash(UseableColor.Purple, Seconds.of(0.2), Seconds.of(0.2)).withTimeout(1))
            .andThen(RobotStatus.setRobotStateOnce(RobotState.Enabled)));

        new Trigger(() -> {
            return endEffector.irSensorTrigger();
        }).onTrue(RobotStatus.setRobotStateOnce(RobotState.ContainingCoral));

        new Trigger(() -> {
            return endEffector.cameraTrigger();
        }).onTrue(RobotStatus.setRobotStateOnce(RobotState.ContainingAlgaeInEE));
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
