package frc.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.superstructure.SuperStructure;
import frc.subsystems.vision.Vision;
import frc.utils.AllianceDependent;
import java.util.List;
import lombok.Getter;
import lombok.Setter;

public class GenericAutomation extends Command {
    public enum AutomationMode {
        PresetOnly, PresetAndAlign
    }

    @Getter
    @Setter private static AutomationMode currentAutomationMode = AutomationMode.PresetOnly;

    public static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength).times(-1);

    public final CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    public final SuperStructure superStructure = SuperStructure.getInstance();

    public Command superStructureCommand;
    public Command driveCommand;
    public Command focusCommand;

    public Pair<Pose2d, Integer> findNearestApriltag(AllianceDependent<List<Pose2d>> tagPostiions,
        List<Pose2d> redTagPoses, List<Pose2d> blueTagPoses, AllianceDependent<int[]> tagSet,
        LimelightsForElements limelights) {
        int tagSeenByBoth = Vision.getInstance().nearestTagToLimelights(limelights);
        if (tagSeenByBoth > 0) {
            // If there is a tag seen by both limelights, use its corresponding pose2d by finding the index of the tag id in the reefIds array
            int[] ids = tagSet.get();
            int i = 0;
            for (int tagId : ids) {
                if (tagId == tagSeenByBoth) break;
                i++;
            }
            if (i < ids.length) {
                return new Pair<Pose2d, Integer>(tagPostiions.get().get(i), tagSeenByBoth);
            }
        }
        Pose2d currentPosition = CommandSwerveDrivetrain.getInstance().getState().Pose;
        Pose2d apriltagPose;
        int tagToFocusOn;
        if (AllianceDependent.isCurrentlyBlue()) {
            apriltagPose = currentPosition.nearest(blueTagPoses);
            tagToFocusOn = tagSet.get()[blueTagPoses.indexOf(apriltagPose)];
        } else {
            apriltagPose = currentPosition.nearest(redTagPoses);
            tagToFocusOn = tagSet.get()[redTagPoses.indexOf(apriltagPose)];
        }
        return new Pair<Pose2d, Integer>(apriltagPose, tagToFocusOn);
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == AutomationMode.PresetOnly) {
            return superStructureCommand.isFinished();
        }
        return driveCommand.isFinished() && superStructureCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (currentAutomationMode == AutomationMode.PresetAndAlign) focusCommand.cancel();
        RobotStatus.setRobotState(RobotState.WaitingToDeAlgae);
    }
}
