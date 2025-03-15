package frc.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.utils.AllianceDependent;
import frc.utils.tuning.TuneableNumber;

public class Dealgae extends GenericAutomation {
    private static final Transform2d Offset = new Transform2d(GenericAutomation.CenterToBumper, Inches.of(0),
        Rotation2d.kZero);
    private static final TuneableNumber DriveToleranceMeters = new TuneableNumber(3, "DeAlgae/DriveTolerance");

    boolean autoSelectDealgae = true;
    boolean manualSelectGettingTop = true;

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
        autoSelectDealgae = true;
    }

    /**
     * Manually select whether to dealgae top or bottom. Disables driveTo.
     */
    public Dealgae(boolean dealgaeTop) {
        autoSelectDealgae = false;
        manualSelectGettingTop = dealgaeTop;
    }

    @Override
    public void initialize() {
        if (autoSelectDealgae) {
            Pair<Pose2d, Integer> apriltagPoseAndId = this.findNearestApriltag(ReefMeasurements.reefApriltagsAlphabetic,
                ReefMeasurements.reefRedApriltags, ReefMeasurements.reefBlueApriltags, ReefMeasurements.reefIds,
                LimelightsForElements.Reef);
            boolean algaeOnTop = algaeOnTopForPose(apriltagPoseAndId.getFirst());

            if (GenericAutomation.getCurrentAutomationMode() == AutomationMode.PresetAndAlign) {
                Pose2d driveToPose = apriltagPoseAndId.getFirst().plus(Offset)
                    .plus(new Transform2d(
                        algaeOnTop ? Presets.TopDADriveOffset.meters() : Presets.BottomDADriveOffset.meters(), 0,
                        Rotation2d.kZero));
                driveCommand = drive.driveTo(driveToPose, DriveToleranceMeters.meters());
                driveCommand.schedule();

                double timeTo = drive.maxTimeToGetToPose(driveToPose);
                superStructureCommand = superStructure.beThereInNoEnd(timeTo,
                    algaeOnTop ? Presets.TopDeAlgae : Presets.BottomDeAlgae);
                System.out.println("Dealgae time to: " + timeTo);

                focusCommand = drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef,
                    apriltagPoseAndId.getSecond());
                focusCommand.schedule();
            } else {
                superStructureCommand = superStructure

                    .beThereAsapNoEnd(algaeOnTop ? Presets.TopDeAlgae : Presets.BottomDeAlgae);
            }
            System.out.println("Auto selected top: " + algaeOnTop);
        } else {
            superStructureCommand = superStructure
                .beThereAsapNoEnd(manualSelectGettingTop ? Presets.TopDeAlgae : Presets.BottomDeAlgae);
            System.out.println("Manual selected top: " + manualSelectGettingTop);
        }
        superStructureCommand.schedule();

        RobotStatus.setRobotState(RobotState.AligningToDeAlgae);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotStatus.setRobotState(RobotState.WaitingToDeAlgae);
    }
}
