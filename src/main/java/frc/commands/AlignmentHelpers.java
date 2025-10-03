package frc.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.vision.Vision;
import frc.utils.AllianceDependent;
import java.util.List;
import lombok.Getter;
import lombok.Setter;

public class AlignmentHelpers {
    public enum AutomationMode {
        PresetOnly, PresetAndAlign
    }

    @Getter
    @Setter private static AutomationMode currentAutomationMode = AutomationMode.PresetOnly;

    public static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength);

    public static final CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();

    public static Pair<Pose2d, Integer> findNearestApriltag(AllianceDependent<List<Pose2d>> tagPostiions,
        List<Pose2d> redTagPoses, List<Pose2d> blueTagPoses, AllianceDependent<int[]> tagSet,
        LimelightsForElements limelights) {
        int tagSeenByAtLeastOne = Vision.getInstance().nearestTagToAtLeastOneOf(limelights);
        if (tagSeenByAtLeastOne > 0) {
            // If there is a tag seen by at least one limelight, use its corresponding pose2d by finding the index of the tag id in the reefIds array
            int[] ids = tagSet.get();
            int i = 0;
            for (int tagId : ids) {
                if (tagId == tagSeenByAtLeastOne) break;
                i++;
            }
            if (i < ids.length) {
                return new Pair<Pose2d, Integer>(tagPostiions.get().get(i), tagSeenByAtLeastOne);
            }
        }
        Pose2d currentPosition = drive.getState().Pose;
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
}
