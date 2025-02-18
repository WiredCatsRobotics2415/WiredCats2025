package frc.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Measurements.RobotMeasurements;
import frc.subsystems.arm.Arm;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.superstructure.SuperStructure;
import lombok.Setter;

/**
 * Command that pathfinds to the nearest side of the reef + a position offset depending on whether you chose right or left, and moves the elevator and the arm to thier presets
 */
public class ScoreCoral extends Command {
    public static enum Side {
        Left, Right
    }

    public static enum Level {
        L1, L2, L3, L4,
    }

    public static enum CoralAutomationMode {
        PresetOnly, PresetAndAlign
    }

    @Setter private static CoralAutomationMode currentAutomationMode = CoralAutomationMode.PresetAndAlign;

    private static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength).times(-1);
    private static final Transform2d LeftOffset = new Transform2d(CenterToBumper, Inches.of(6), new Rotation2d());
    private static final Transform2d RightOffset = new Transform2d(CenterToBumper, Inches.of(-6), new Rotation2d());
    private static final double DriveToleranceMeters = Units.inchesToMeters(5);

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private SuperStructure superStructure = SuperStructure.getInstance();

    private double goalHeightInches;
    private double goalArmDegrees;
    private Transform2d offset;

    private Command driveCommand;
    private Command superStructureCommand;

    private Pose2d findNearestReefSideApriltag() {
        Pose2d currentPosition = CommandSwerveDrivetrain.getInstance().getState().Pose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return currentPosition.nearest(ReefMeasurements.reefBlueApriltags);
        } else {
            return currentPosition.nearest(ReefMeasurements.reefRedApriltags);
        }
    }

    public ScoreCoral(Side reefSide, Level reefLevel) {
        // TODO: Note that if the robot is not driveable while the preset is going even though
        // You are in the PresetOnly mode, consider moving this addRequirements line to
        // initialize so it doesn't require drive too early
        addRequirements(Elevator.getInstance(), Arm.getInstance());

        switch (reefLevel) {
            case L1:
                goalHeightInches = Presets.Level1Height.in(Inches);
                goalArmDegrees = Presets.Level1Angle.in(Degrees);
                break;
            case L2:
                goalHeightInches = Presets.Level2Height.in(Inches);
                goalArmDegrees = Presets.Level2Angle.in(Degrees);
                break;
            case L3:
                goalHeightInches = Presets.Level3Height.in(Inches);
                goalArmDegrees = Presets.Level3Angle.in(Degrees);
                break;
            case L4:
                goalHeightInches = Presets.Level4Height.in(Inches);
                goalArmDegrees = Presets.Level4Angle.in(Degrees);
                break;
            default:
                goalHeightInches = Presets.Level1Height.in(Inches);
                goalArmDegrees = Presets.Level1Angle.in(Degrees);
                break;
        }

        offset = reefSide.equals(Side.Left) ? LeftOffset : RightOffset;
    }

    @Override
    public void initialize() {
        System.out.println("test initialize 2");
        superStructureCommand = superStructure.runToPositionCommand(Inches.of(goalHeightInches),
            Degrees.of(goalArmDegrees));
        System.out.println("scheduled super structure command");
        superStructureCommand.schedule();

        if (currentAutomationMode == CoralAutomationMode.PresetAndAlign) {
            Pose2d driveTo = findNearestReefSideApriltag().plus(offset);
            driveCommand = drive.driveTo(driveTo, DriveToleranceMeters);
            System.out.println("BEFORE scheduled drive command");
            driveCommand.schedule();
            System.out.println("AFTER scheduled drive command");
        }
    }

    @Override
    public void execute() {
        System.out.println(driveCommand.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ended" + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == CoralAutomationMode.PresetOnly) {
            return superStructureCommand.isFinished();
        }
        return driveCommand.isFinished() && superStructureCommand.isFinished();
    }
}
