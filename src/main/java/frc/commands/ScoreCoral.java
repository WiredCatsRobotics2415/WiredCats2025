package frc.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.superstructure.SuperStructure;
import frc.subsystems.superstructure.TuneableSuperStructureState;
import frc.subsystems.vision.Vision;
import frc.utils.AllianceDependent;
import frc.utils.tuning.TuneableDistance;
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
    private static final TuneableDistance LeftOffset = new TuneableDistance(6, "ScoreCoral/LeftOffset");
    private static final TuneableDistance RightOffset = new TuneableDistance(6, "ScoreCoral/RightOffset");
    private static final TuneableDistance DriveToleranceMeters = new TuneableDistance(3, "ScoreCoral/DriveTolerance");

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private SuperStructure superStructure = SuperStructure.getInstance();

    private TuneableSuperStructureState superStructureState;
    private TuneableDistance goalDriveOffset;
    private Side side;
    private int tagToFocusOn = -1;

    private Command driveCommand;
    private Command focusCommand;
    private Command superStructureCommand;

    public Pose2d findNearestReefSideApriltag() {
        int tagSeenByBoth = Vision.getInstance().nearestTagToLimelights(LimelightsForElements.Reef);
        if (tagSeenByBoth > 0) {
            // If there is a tag seen by both limelights, use its corresponding pose2d by finding the index of the tag id in the reefIds array
            int[] reefIds = ReefMeasurements.reefIds.get();
            int i = 0;
            for (int tagId : reefIds) {
                if (tagId == tagSeenByBoth) break;
                i++;
            }
            if (i < reefIds.length) {
                tagToFocusOn = tagSeenByBoth;
                return ReefMeasurements.reefApriltagsAlphabetic.get().get(i);
            }
        }
        Pose2d currentPosition = CommandSwerveDrivetrain.getInstance().getState().Pose;
        Pose2d apriltagPose;
        if (AllianceDependent.isCurrentlyBlue()) {
            apriltagPose = currentPosition.nearest(ReefMeasurements.reefBlueApriltags);
            tagToFocusOn = ReefMeasurements.reefBlueIds[ReefMeasurements.reefBlueApriltags.indexOf(apriltagPose)];
        } else {
            apriltagPose = currentPosition.nearest(ReefMeasurements.reefRedApriltags);
            tagToFocusOn = ReefMeasurements.reefRedIds[ReefMeasurements.reefRedApriltags.indexOf(apriltagPose)];
        }
        return apriltagPose;
    }

    public ScoreCoral(Side reefSide, Level reefLevel) {
        // addRequirements(SuperStructure.getInstance());
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
        if (currentAutomationMode == CoralAutomationMode.PresetAndAlign) {
            Transform2d leftOffset = new Transform2d(CenterToBumper.plus(goalDriveOffset.distance()),
                LeftOffset.distance(), Rotation2d.kZero);
            Transform2d rightOffset = new Transform2d(CenterToBumper.plus(goalDriveOffset.distance()),
                RightOffset.distance().times(-1), Rotation2d.kZero);
            Transform2d offset = side.equals(Side.Left) ? leftOffset : rightOffset;
            Pose2d driveTo = findNearestReefSideApriltag().plus(offset);
            driveCommand = drive.driveTo(driveTo, DriveToleranceMeters.in(Meters));
            driveCommand.schedule();

            double timeTo = drive.maxTimeToGetToPose(driveTo);
            superStructureCommand = superStructure.beThereIn(timeTo, superStructureState);
            System.out.println("Score coral time to: " + timeTo);

            focusCommand = drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, tagToFocusOn);
            focusCommand.schedule();
        } else {
            superStructureCommand = superStructure.beThereAsap(superStructureState);
        }
        superStructureCommand.schedule();

        RobotStatus.setRobotState(RobotState.AligningToScoreCoral);
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == CoralAutomationMode.PresetOnly) {
            return SuperStructure.getInstance().allAtGoal();
        }
        System.out
            .println("finished? " + driveCommand.isFinished() + " && " + SuperStructure.getInstance().allAtGoal());
        return driveCommand.isFinished() && SuperStructure.getInstance().allAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        if (currentAutomationMode == CoralAutomationMode.PresetAndAlign) focusCommand.cancel();
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
