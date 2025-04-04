package frc.robot;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.autos.CtoRGround;
import frc.autos.CtoRHPS;
import frc.autos.GenericAuto;
import frc.autos.L4;
import frc.autos.L4AndDealgae;
import frc.commands.AlignToHPS;
import frc.commands.AlignToHPS.HPSSide;
import frc.commands.AlignToReef;
import frc.commands.AlignToReef.Side;
import frc.commands.DealgaePresetTo;
import frc.commands.InBargeZoneAlert;
import frc.commands.ReefPresetTo;
import frc.commands.ReefPresetTo.Level;
import frc.constants.Controls;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Subsystems.DriveConstants;
import frc.constants.Subsystems.LEDStripConstants.UseableColor;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.subsystems.arm.Arm;
import frc.subsystems.climb.Climb;
import frc.subsystems.coralintake.CoralIntake;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.drive.CommandSwerveDrivetrain.PoseEstimationType;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.leds.LEDStrip;
import frc.subsystems.superstructure.SuperStructure;
import frc.subsystems.vision.Vision;
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
    private Climb climber = Climb.getInstance();

    private LoggedDashboardChooser<Command> autoChooser;

    public enum TeleopDriveMode {
        Normal, MinorAdjustment
    }

    public enum AligningTo {
        Reef, HPS, CenterReefForDealgae
    }

    @Getter private TeleopDriveMode currentTeleopDriveMode = TeleopDriveMode.Normal;
    @Getter private AligningTo currentlyAligningTo = AligningTo.Reef;

    private AdjustableSLR driveXLimiter = new AdjustableSLR(DriveConstants.BaseXAccelerationMax.get());
    private AdjustableSLR driveYLimiter = new AdjustableSLR(DriveConstants.BaseYAccelerationMax.get());
    private AdjustableSLR driveRotationLimiter = new AdjustableSLR(DriveConstants.BaseRotationAccelMax.get());
    private TuneableNumber minorAdjXPct = new TuneableNumber(0.6, "Drive/minorAdjXPct");
    private TuneableNumber minorAdjYPct = new TuneableNumber(0.6, "Drive/minorAdjYPct");

    private ParallelRaceGroup alignToReefLeft = new AlignToReef(Side.Left).withTimeout(10);
    private ParallelRaceGroup alignToReefRight = new AlignToReef(Side.Right).withTimeout(10);
    private ParallelRaceGroup alignToHPSLeft = new AlignToHPS(HPSSide.Left).withTimeout(10);
    private ParallelRaceGroup alignToHPSRight = new AlignToHPS(HPSSide.Right).withTimeout(10);
    private ParallelRaceGroup alignToReefDealgae = new AlignToReef(Side.Center, Presets.DealgaeDriveOffset)
        .withTimeout(10);
    private boolean runningAutoAlign = false;
    private TuneableNumber rawJoyAboveThresholdToCancelAutoAlign = new TuneableNumber(0.1,
        "RobotContainer/rawJoyAboveThresholdToCancelAutoAlign");

    private boolean manualGroundInakeOnGround = false;

    private int autoChangerCommandLastHC = 0;

    private RobotContainer() {
        setupAuto();
        configureControls();
        configureTriggers();
        neutralizeSubsystems();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void setupAuto() {
        // Put Auto named commands here
        NamedCommands.registerCommand("L4", new ReefPresetTo(Level.L4));
        NamedCommands.registerCommand("L3", new ReefPresetTo(Level.L3));
        NamedCommands.registerCommand("L2", new ReefPresetTo(Level.L2));
        NamedCommands.registerCommand("L1", new ReefPresetTo(Level.L1));
        // these are not right... i need to see how exactly we need to move subsystems to do ground and source intake
        NamedCommands.registerCommand("GroundIntake", superstructure.beThereAsap(Presets.GroundIntake, false, false)
            .alongWith(endEffector.intakeAndWaitForCoral()));
        NamedCommands.registerCommand("SourceIntake", superstructure.beThereAsap(Presets.IntakeFromHPS, false, false)
            .alongWith(endEffector.intakeAndWaitForCoral()));
        NamedCommands.registerCommand("FocusPEOnReef",
            drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, ReefMeasurements.reefIds));
        NamedCommands.registerCommand("SwitchToGlobalPE",
            new InstantCommand(() -> drive.switchPoseEstimator(PoseEstimationType.Global)));
        // NamedCommands.registerCommand("align", vision.singleTagTrack());
        // named commands for 2025 gwinnett
        NamedCommands.registerCommand("Outtake", endEffector.toggleOuttakeCoral());
        NamedCommands.registerCommand("RaiseUp", new InstantCommand(() -> elevator.setGoal(40)));

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

        autoChooser.addOption("CustomL4", new L4());
        autoChooser.addOption("CustomCtoRHPS", new CtoRHPS());
        autoChooser.addOption("CtoRGround", new CtoRGround());
        autoChooser.addOption("L4AndDealgae", new L4AndDealgae());
    }

    public void teleopEnable() {
        neutralizeSubsystems();
    }

    private void configureControls() {
        oi.binds.get(OI.Bind.SeedFieldCentric)
            .onTrue(drive.resetRotationFromLimelightMT1().ignoringDisable(true).withTimeout(1));

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
                    .withRotationalRate(driveRotationLimiter.calculate(0));
            }
            double[] linearInput = oi.getXY();
            double x = linearInput[1], y = linearInput[0];
            double rotation = oi.getRotation();
            return drive.driveOpenLoopFieldCentricRequest
                .withVelocityX(driveXLimiter.calculate(-x * Controls.MaxDriveMeterS))
                .withVelocityY(driveYLimiter.calculate(-y * Controls.MaxDriveMeterS))
                .withRotationalRate(driveRotationLimiter.calculate(-rotation * Controls.MaxAngularRadS));
        }).withName("Teleop Default"));
        oi.binds.get(OI.Bind.ChangeTeleopMode).debounce(0.25, DebounceType.kRising)
            .onTrue(Commands.runOnce(() -> currentTeleopDriveMode = TeleopDriveMode.MinorAdjustment))
            .onFalse(Commands.runOnce(() -> currentTeleopDriveMode = TeleopDriveMode.Normal));

        oi.binds.get(OI.Bind.ManualElevatorUp)
            .whileTrue(Commands.run(() -> superstructure.changeElevatorGoalSafely(0.25)));
        oi.binds.get(OI.Bind.ManualElevatorDown)
            .whileTrue(Commands.run(() -> superstructure.changeElevatorGoalSafely(-0.25)));

        oi.binds.get(OI.Bind.ManualArmForward).whileTrue(Commands.run(() -> superstructure.changeArmGoalSafely(-1.0)));
        oi.binds.get(OI.Bind.ManualArmBack).whileTrue(Commands.run(() -> superstructure.changeArmGoalSafely(1.0)));

        oi.binds.get(OI.Bind.Shoot)
            .onTrue(endEffector.toggleOuttakeCoral().alongWith(CoralIntake.getInstance().toggleOuttake()))
            .onFalse(endEffector.turnOff().alongWith(CoralIntake.getInstance().turnOffRollers()));
        oi.binds.get(OI.Bind.DeAlgae).onTrue(endEffector.toggleIntakeAlgae());
        oi.binds.get(OI.Bind.ManualIntake)
            .onTrue(endEffector.toggleIntakeCoral().alongWith(CoralIntake.getInstance().toggleIntake()));
        oi.binds.get(OI.Bind.ManualAlgaeShoot).onTrue(endEffector.toggleOuttakeAlgae());
        // In case it should be A intakes coral, outtakes algae and B vice versa...
        // Dealgae is button A:
        // oi.binds.get(OI.Bind.DeAlgae).onTrue(endEffector.toggleIntakeCoral().alongWith(coralIntake.toggleIntake()))
        // .onFalse(endEffector.turnOff().alongWith(coralIntake.turnOffRollers()));
        // Shoot is button B:
        // oi.binds.get(OI.Bind.DeAlgae).onTrue(endEffector.toggleOuttake().alongWith(coralIntake.toggleOuttake())).onFalse(endEffector.turnOff().alongWith(coralIntake.turnOffRollers()));

        // oi.binds.get(OI.Bind.DeStickArm).onTrue(arm.deStuck());
        oi.binds.get(OI.Bind.StowPreset).onTrue(superstructure.stow().ignoringDisable(true));
        oi.binds.get(OI.Bind.IntakeFromHPS).onTrue(superstructure.beThereAsapNoEnd(Presets.IntakeFromHPS, false, false)
            .alongWith(changeAlignTarget(AligningTo.HPS)));
        oi.binds.get(OI.Bind.DealgaePresetTop).onTrue(new DealgaePresetTo(true)
            .alongWith(changeAlignTarget(AligningTo.CenterReefForDealgae)).alongWith(endEffector.toggleIntakeAlgae()));
        oi.binds.get(OI.Bind.DealgaePresetBottom).onTrue(new DealgaePresetTo(false)
            .alongWith(changeAlignTarget(AligningTo.CenterReefForDealgae)).alongWith(endEffector.toggleIntakeAlgae()));;
        oi.binds.get(OI.Bind.L1).onTrue(new ReefPresetTo(Level.L1).alongWith(changeAlignTarget(AligningTo.Reef)));
        oi.binds.get(OI.Bind.L2).onTrue(new ReefPresetTo(Level.L2).alongWith(changeAlignTarget(AligningTo.Reef)));
        oi.binds.get(OI.Bind.L3).onTrue(new ReefPresetTo(Level.L3).alongWith(changeAlignTarget(AligningTo.Reef)));
        oi.binds.get(OI.Bind.L4).onTrue(new ReefPresetTo(Level.L4).alongWith(changeAlignTarget(AligningTo.Reef)));
        oi.binds.get(OI.Bind.StackAlageIntake)
            .onTrue(superstructure.beThereAsapNoEnd(Presets.StackIntakeAlgae, false, false));
        oi.binds.get(OI.Bind.AutoAlignLeft).onTrue(Commands.runOnce(() -> {
            System.out.println("Left auto align is aligning to: " + currentlyAligningTo);
            switch (currentlyAligningTo) {
                case Reef:
                    alignToReefLeft.schedule();
                    break;
                case HPS:
                    alignToHPSLeft.schedule();
                    break;
                case CenterReefForDealgae:
                    alignToReefDealgae.schedule();
                    break;
                default:
                    alignToReefLeft.schedule();
                    break;
            }
            runningAutoAlign = true;
        }));
        oi.binds.get(OI.Bind.AutoAlignRight).onTrue(Commands.runOnce(() -> {
            switch (currentlyAligningTo) {
                case Reef:
                    alignToReefRight.schedule();
                    break;
                case HPS:
                    alignToHPSRight.schedule();
                    break;
                case CenterReefForDealgae:
                    alignToReefDealgae.schedule();
                    break;
                default:
                    alignToReefRight.schedule();
                    break;
            }
            runningAutoAlign = true;
        }));

        // oi.binds.get(OI.Bind.AutoIntakeFromGround)
        // .onTrue(Commands.runOnce(() -> vision.setEndEffectorPipeline(Vision.EndEffectorPipeline.NeuralNetwork))
        // .alongWith(superstructure.beThereAsapNoEnd(Presets.GroundIntake))
        // .alongWith(Commands.waitSeconds(1.5).andThen(new AutoIntake())))
        // .onFalse(superstructure.stow().andThen(CoralIntake.getInstance().turnOffRollers())
        // .finallyDo(() -> vision.setEndEffectorPipeline(Vision.EndEffectorPipeline.DriverView)));

        oi.binds.get(OI.Bind.AutoIntakeFromGround)
            .onTrue(superstructure.beThereAsapNoEnd(Presets.GroundIntake, false, false).alongWith(Commands.runOnce(() ->
            {
                endEffector.intakeCoral().schedule();
                CoralIntake.getInstance().intake().schedule();
            })))
            .onFalse(new ParallelCommandGroup(
                superstructure.beThereAsap(Presets.GroundIntakeUp, false, false).andThen(Commands.waitSeconds(0.5))
                    .andThen(superstructure.stow()),
                endEffector.turnOff(), CoralIntake.getInstance().turnOffRollers()));

        // oi.binds.get(OI.Bind.AutoIntakeFromGround)
        // .onTrue(superstructure.beThereAsapNoEnd(Presets.GroundIntake).alongWith(Commands.waitSeconds(1))
        // .andThen(CoralIntake.getInstance().toggleIntake()).alongWith(endEffector.toggleIntakeCoral()))
        // .onFalse(superstructure.stow());

        oi.binds.get(OI.Bind.ProcessorPreset)
            .onTrue(superstructure.beThereAsapNoEnd(Presets.ProcessorScore, true, true));
        oi.binds.get(OI.Bind.GroundIntakeAlgae).onTrue(superstructure
            .beThereAsapNoEnd(Presets.GroundIntakeAlgae, false, false).alongWith(endEffector.intakeAlgae()));
        oi.binds.get(OI.Bind.BargePreset).onTrue(superstructure.beThereAsapNoEnd(Presets.Barge, true, true));

        oi.binds.get(OI.Bind.GroundPound).onTrue(Commands.runOnce(() -> {
            elevator.setVoltageOverride(true);
            elevator.getIo().setVoltage(2);
        }, elevator).andThen(Commands.waitSeconds(0.5)).andThen(Commands.runOnce(() -> {
            elevator.getIo().setVoltage(-1.5);
        }, elevator)).andThen(Commands.waitSeconds(0.75)).andThen(Commands.runOnce(() -> {
            elevator.setVoltageOverride(false);
        }, elevator)));
    }

    private Command changeAlignTarget(AligningTo target) {
        return Commands.runOnce(() -> {
            System.out.println("Changed align target to: " + target.toString());
            this.currentlyAligningTo = target;
        });
    }

    private void cancelAutoAlignment() {
        alignToHPSLeft.cancel();
        alignToHPSRight.cancel();
        alignToReefLeft.cancel();
        alignToReefRight.cancel();
        alignToReefDealgae.cancel();
    }

    private void configureTriggers() {
        // If you change this remember to change the start condition in Vision constructor
        new Trigger(() -> {
            return arm.getMeasurement() > 90.0d;
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

        new InBargeZoneAlert().schedule();
    }

    public void periodic() {
        double[] ssLimits = superstructure.recommendedDriveAccelLimits();
        // driveXLimiter.setRateLimit(ssLimits[0]);
        // driveYLimiter.setRateLimit(ssLimits[1]);
        // driveRotationLimiter.setRateLimit(ssLimits[2]);

        drive.getDriveToPositionXController()
            .setConstraints(new Constraints(DriveConstants.BaseVelocityMax.get(), ssLimits[0]));
        drive.getDriveToPositionYController()
            .setConstraints(new Constraints(DriveConstants.BaseVelocityMax.get(), ssLimits[1]));

        if (runningAutoAlign) {
            double[] input = oi.getRawXY();
            if (Math.abs(input[0]) > rawJoyAboveThresholdToCancelAutoAlign.get()
                || Math.abs(input[1]) > rawJoyAboveThresholdToCancelAutoAlign.get()) {
                System.out.println("Robot container: canceled auto input");
                cancelAutoAlignment();
                runningAutoAlign = false;
            }
        }

        Command c = autoChooser.get();
        if (c != null) {
            int currentHC = c.hashCode();
            if (currentHC != autoChangerCommandLastHC) {
                autoChangerCommandLastHC = currentHC;
                if (c instanceof GenericAuto) {
                    System.out.println("Setting auto paths");
                    ((GenericAuto) c).addPaths();
                }
            }
        }

        Logger.recordOutput("RobotContainer/TeleopMode", currentTeleopDriveMode.toString());
        Logger.recordOutput("RobotContainer/AligningTo", currentlyAligningTo.toString());
    }

    public Command getAutonomousCommand() { return autoChooser.get(); }

    public void neutralizeSubsystems() {
        arm.setGoal(arm.getMeasurement());
        elevator.setGoal(elevator.getMeasurement());
        CoralIntake.getInstance().setPivotGoal(CoralIntake.getInstance().getPivotAngle());
    }
}
