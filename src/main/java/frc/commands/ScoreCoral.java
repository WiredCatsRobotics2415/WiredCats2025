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
import frc.utils.tuning.TuneableNumber;
import lombok.Getter;
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

    @Setter
    @Getter private static CoralAutomationMode currentAutomationMode = CoralAutomationMode.PresetAndAlign;

    private static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength).times(-1);
    private static final TuneableNumber LeftRightOffset = new TuneableNumber(6, "ScoreCoral/LROffsetInches");
    private static final double DriveToleranceMeters = Units.inchesToMeters(5);

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private SuperStructure superStructure = SuperStructure.getInstance();

    private double goalHeightInches;
    private double goalArmDegrees;
    private Distance goalDriveOffset;
    private Side side;

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
        addRequirements(Elevator.getInstance(), Arm.getInstance());
        side = reefSide;

        switch (reefLevel) {
            case L1:
                goalHeightInches = Presets.Level1Height.in(Inches);
                goalArmDegrees = Presets.Level1Angle.in(Degrees);
                goalDriveOffset = Presets.Level1DriveOffset;
                break;
            case L2:
                goalHeightInches = Presets.Level2Height.in(Inches);
                goalArmDegrees = Presets.Level2Angle.in(Degrees);
                goalDriveOffset = Presets.Level2DriveOffset;
                break;
            case L3:
                goalHeightInches = Presets.Level3Height.in(Inches);
                goalArmDegrees = Presets.Level3Angle.in(Degrees);
                goalDriveOffset = Presets.Level3DriveOffset;
                break;
            case L4:
                goalHeightInches = Presets.Level4Height.in(Inches);
                goalArmDegrees = Presets.Level4Angle.in(Degrees);
                goalDriveOffset = Presets.Level4DriveOffset;
                break;
            default:
                goalHeightInches = Presets.Level1Height.in(Inches);
                goalArmDegrees = Presets.Level1Angle.in(Degrees);
                goalDriveOffset = Presets.Level1DriveOffset;
                break;
        }
    }

    @Override
    public void initialize() {
        superStructureCommand = superStructure.runToPositionCommand(Inches.of(goalHeightInches),
            Degrees.of(goalArmDegrees));
        superStructureCommand.schedule();

        if (currentAutomationMode == CoralAutomationMode.PresetAndAlign) {
            Transform2d leftOffset = new Transform2d(CenterToBumper.plus(goalDriveOffset),
                Inches.of(LeftRightOffset.get()), new Rotation2d());
            Transform2d rightOffset = new Transform2d(CenterToBumper.plus(goalDriveOffset),
                Inches.of(-LeftRightOffset.get()), new Rotation2d());
            Transform2d offset = side.equals(Side.Left) ? leftOffset : rightOffset;
            Pose2d driveTo = findNearestReefSideApriltag().plus(offset);
            driveCommand = drive.driveTo(driveTo, DriveToleranceMeters);
            driveCommand.schedule();
        }
    }

    @Override
    public void execute() {
        if (driveCommand != null) System.out.println(driveCommand.isFinished());
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == CoralAutomationMode.PresetOnly) {
            return superStructureCommand.isFinished();
        }
        return driveCommand.isFinished() && superStructureCommand.isFinished();
    }
}
