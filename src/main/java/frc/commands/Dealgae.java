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
import frc.subsystems.vision.Vision;
import frc.utils.AllianceDependent;
import frc.utils.tuning.TuneableDistance;
import lombok.Getter;
import lombok.Setter;

public class Dealgae extends Command {
    public static enum DealgaeAutomationMode {
        PresetOnly, PresetAndAlign
    }

    @Setter
    @Getter private static DealgaeAutomationMode currentAutomationMode = DealgaeAutomationMode.PresetAndAlign;

    private static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength).times(-1);
    private static final Transform2d Offset = new Transform2d(CenterToBumper, Inches.of(0), Rotation2d.kZero);
    private static final TuneableDistance DriveToleranceMeters = new TuneableDistance(3, "DeAlgae/DriveTolerance");

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private SuperStructure superStructure = SuperStructure.getInstance();

    private int tagToFocusOn = -1;

    private Command driveCommand;
    private Command superStructureCommand;
    private Command focusCommand;

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

    private boolean algaeOnTopForPose(Pose2d apriltagPose) {
        if (AllianceDependent.isCurrentlyBlue()) {
            return ReefMeasurements.ReefAlgaeOnTopAlphabeticOrder
                .get(ReefMeasurements.reefBlueApriltags.indexOf(apriltagPose));
        } else {
            return ReefMeasurements.ReefAlgaeOnTopAlphabeticOrder
                .get(ReefMeasurements.reefRedApriltags.indexOf(apriltagPose));
        }
    }

    public Dealgae() {
        // addRequirements(SuperStructure.getInstance());
    }

    @Override
    public void initialize() {
        Pose2d apriltagPose = findNearestReefSideApriltag();
        boolean algaeOnTop = algaeOnTopForPose(apriltagPose);

        if (currentAutomationMode == DealgaeAutomationMode.PresetAndAlign) {
            Pose2d driveToPose = apriltagPose.plus(Offset)
                .plus(new Transform2d(
                    algaeOnTop ? Presets.TopDADriveOffset.distance() : Presets.BottomDADriveOffset.distance(),
                    Inches.of(0), Rotation2d.kZero));
            driveCommand = drive.driveTo(driveToPose, DriveToleranceMeters.in(Meters));
            driveCommand.schedule();

            double timeTo = drive.maxTimeToGetToPose(driveToPose);
            superStructureCommand = superStructure.beThereIn(timeTo,
                algaeOnTop ? Presets.TopDeAlgae : Presets.BottomDeAlgae);
            System.out.println("Dealgae time to: " + timeTo);

            focusCommand = drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, tagToFocusOn);
            focusCommand.schedule();
        } else {
            superStructureCommand = superStructure.beThereAsap(algaeOnTop ? Presets.TopDeAlgae : Presets.BottomDeAlgae);
        }
        superStructureCommand.schedule();

        RobotStatus.setRobotState(RobotState.AligningToDeAlgae);
    }

    @Override
    public void execute() {
        if (driveCommand != null) System.out.println(driveCommand.isFinished());
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == DealgaeAutomationMode.PresetOnly) {
            return superStructureCommand.isFinished();
        }
        return driveCommand.isFinished() && superStructureCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (currentAutomationMode == DealgaeAutomationMode.PresetAndAlign) focusCommand.cancel();
        RobotStatus.setRobotState(RobotState.WaitingToDeAlgae);
    }
}
