package frc.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commands.Dealgae.DealgaeAutomationMode;
import frc.commands.ScoreCoral.CoralAutomationMode;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.HumanPlayerStation;
import frc.constants.Measurements.RobotMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.subsystems.endeffector.EndEffector;
import frc.subsystems.superstructure.SuperStructure;
import frc.subsystems.vision.Vision;
import frc.utils.AllianceDependent;
import frc.utils.tuning.TuneableDistance;
import lombok.Getter;
import lombok.Setter;

public class IntakeFromHPS extends Command {
    public static enum HPSAutomationMode {
        PresetOnly, PresetAndAlign
    }

    @Setter
    @Getter private static HPSAutomationMode currentAutomationMode = HPSAutomationMode.PresetAndAlign;

    private static final Distance CenterToBumper = RobotMeasurements.CenterToFramePerpendicular
        .plus(RobotMeasurements.BumperLength).times(-1);
        private static final Transform2d Offset = new Transform2d(CenterToBumper, Inches.of(0), Rotation2d.kZero);
    private static final TuneableDistance DriveToleranceMeters = new TuneableDistance(3, "IntakeFromHPS/DriveTolerance");

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private SuperStructure superStructure = SuperStructure.getInstance();
    
    private int tagToFocusOn = -1;

    private Command driveCommand;
    private Command focusCommand;
    private Command superStructureCommand;

    public Pose2d findNearestHPSApriltag() {
        int tagSeenByBoth = Vision.getInstance().nearestTagToLimelights(LimelightsForElements.HumanPlayerStation);
        if (tagSeenByBoth > 0) {
            // If there is a tag seen by both limelights, use its corresponding pose2d by finding the index of the tag id in the hpsIds array
            int[] hpsIds = HumanPlayerStation.hpsIds.get();
            int i = 0;
            for (int tagId : hpsIds) {
                if (tagId == tagSeenByBoth) break;
                i++;
            }
            if (i < hpsIds.length) {
                tagToFocusOn = tagSeenByBoth;
                return HumanPlayerStation.hpsApriltags.get().get(i);
            }
        }
        Pose2d currentPosition = CommandSwerveDrivetrain.getInstance().getState().Pose;
        Pose2d apriltagPose;
        if (AllianceDependent.isCurrentlyBlue()) {
            apriltagPose = currentPosition.nearest(HumanPlayerStation.hpsBlueApriltags);
            tagToFocusOn = HumanPlayerStation.hpsBlueIds[HumanPlayerStation.hpsBlueApriltags.indexOf(apriltagPose)];
        } else {
            apriltagPose = currentPosition.nearest(HumanPlayerStation.hpsRedApriltags);
            tagToFocusOn = HumanPlayerStation.hpsRedIds[HumanPlayerStation.hpsRedApriltags.indexOf(apriltagPose)];
        }
        return apriltagPose;
    }

    @Override
    public void initialize() {
        Pose2d apriltagPose = findNearestHPSApriltag();

        if (currentAutomationMode == HPSAutomationMode.PresetAndAlign) {
            Pose2d driveToPose = apriltagPose.plus(Offset)
                .plus(new Transform2d(Presets.HPSDriveOffset.distance(), Inches.of(0), Rotation2d.kZero));
            driveCommand = drive.driveTo(driveToPose, DriveToleranceMeters.in(Meters));
            driveCommand.schedule();

            double timeTo = drive.maxTimeToGetToPose(driveToPose);
            superStructureCommand = superStructure.beThereIn(timeTo, Presets.IntakeFromHPS);
            System.out.println("HPS time to: " + timeTo);

            focusCommand = drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, tagToFocusOn);
            focusCommand.schedule();
        } else {
            superStructureCommand = superStructure.beThereAsap(Presets.IntakeFromHPS);
        }
        superStructureCommand.schedule();
        EndEffector.getInstance().intakeAndWaitForCoral().schedule();

        RobotStatus.setRobotState(RobotState.AligningToHPS);
    }

    @Override
    public void execute() {
        if (driveCommand != null) System.out.println(driveCommand.isFinished());
    }

    @Override
    public boolean isFinished() {
        if (currentAutomationMode == HPSAutomationMode.PresetOnly) {
            return superStructureCommand.isFinished();
        }
        return driveCommand.isFinished() && superStructureCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (currentAutomationMode == HPSAutomationMode.PresetAndAlign) focusCommand.cancel();
        RobotStatus.setRobotState(RobotState.WaitingForCoralAtHPS);
    }
}
