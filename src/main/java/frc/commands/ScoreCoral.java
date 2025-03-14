package frc.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.commands.GenericAutomation.AutomationMode;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.superstructure.TuneableSuperStructureState;
import frc.utils.tuning.TuneableNumber;

/**
 * Command that pathfinds to the nearest side of the reef + a position offset depending on whether you chose right or left, and moves the elevator and the arm to thier presets
 */
public class ScoreCoral extends GenericAutomation {
    public static enum Side {
        Left, Right
    }

    public static enum Level {
        L1, L2, L3, L4,
    }

    private static final TuneableNumber LeftOffset = new TuneableNumber(6, "ScoreCoral/LeftOffset");
    private static final TuneableNumber RightOffset = new TuneableNumber(6, "ScoreCoral/RightOffset");
    private static final TuneableNumber DriveToleranceMeters = new TuneableNumber(3, "ScoreCoral/DriveTolerance");

    private TuneableSuperStructureState superStructureState;
    private TuneableNumber goalDriveOffset;
    private Side side;

    public ScoreCoral(Side reefSide, Level reefLevel) {
        side = reefSide;

        switch (reefLevel) {
            case L1:
                superStructureState = Presets.Level1;
                goalDriveOffset = Presets.Level1DriveOffset;
                break;
            case L2:
                superStructureState = Presets.Level2;
                goalDriveOffset = Presets.Level2DriveOffset;
                break;
            case L3:
                superStructureState = Presets.Level3;
                goalDriveOffset = Presets.Level3DriveOffset;
                break;
            case L4:
                superStructureState = Presets.Level4;
                goalDriveOffset = Presets.Level4DriveOffset;
                break;
            default:
                superStructureState = Presets.Level1;
                goalDriveOffset = Presets.Level1DriveOffset;
                break;
        }
    }

    @Override
    public void initialize() {
        if (GenericAutomation.getCurrentAutomationMode() == AutomationMode.PresetAndAlign) {
            Transform2d leftOffset = new Transform2d(GenericAutomation.CenterToBumper.plus(goalDriveOffset.distance()),
                LeftOffset.distance(), Rotation2d.kZero);
            Transform2d rightOffset = new Transform2d(GenericAutomation.CenterToBumper.plus(goalDriveOffset.distance()),
                RightOffset.distance().times(-1), Rotation2d.kZero);
            Transform2d offset = side.equals(Side.Left) ? leftOffset : rightOffset;
            Pair<Pose2d, Integer> apriltagPoseAndId = this.findNearestApriltag(ReefMeasurements.reefApriltagsAlphabetic,
                ReefMeasurements.reefRedApriltags, ReefMeasurements.reefBlueApriltags, ReefMeasurements.reefIds,
                LimelightsForElements.Reef);
            Pose2d driveTo = apriltagPoseAndId.getFirst().plus(offset);
            driveCommand = drive.driveTo(driveTo, DriveToleranceMeters.meters());
            driveCommand.schedule();

            double timeTo = drive.maxTimeToGetToPose(driveTo);
            superStructureCommand = superStructure.beThereIn(timeTo, superStructureState);
            System.out.println("Score coral time to: " + timeTo);

            focusCommand = drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef,
                apriltagPoseAndId.getSecond());
            focusCommand.schedule();
        } else {
            superStructureCommand = superStructure.beThereAsap(superStructureState);
        }
        superStructureCommand.schedule();

        RobotStatus.setRobotState(RobotState.AligningToScoreCoral);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
