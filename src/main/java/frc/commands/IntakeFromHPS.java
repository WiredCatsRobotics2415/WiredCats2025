package frc.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.HumanPlayerStation;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.endeffector.EndEffector;
import frc.utils.tuning.TuneableNumber;

public class IntakeFromHPS extends GenericAutomation {
    private static final Transform2d Offset = new Transform2d(GenericAutomation.CenterToBumper, Inches.of(0),
        Rotation2d.kZero);
    private static final TuneableNumber DriveToleranceMeters = new TuneableNumber(3, "IntakeFromHPS/DriveTolerance");

    @Override
    public void initialize() {
        Pair<Pose2d, Integer> apriltagPoseAndId = this.findNearestApriltag(HumanPlayerStation.hpsApriltags,
            HumanPlayerStation.hpsRedApriltags, HumanPlayerStation.hpsBlueApriltags, HumanPlayerStation.hpsIds,
            LimelightsForElements.HumanPlayerStation);

        if (GenericAutomation.getCurrentAutomationMode() == AutomationMode.PresetAndAlign) {
            Pose2d driveToPose = apriltagPoseAndId.getFirst().plus(Offset)
                .plus(new Transform2d(Presets.HPSDriveOffset.meters(), 0, Rotation2d.kZero));
            driveCommand = drive.driveTo(driveToPose, DriveToleranceMeters.meters());
            driveCommand.schedule();

            double timeTo = drive.maxTimeToGetToPose(driveToPose);
            superStructureCommand = superStructure.beThereIn(timeTo, Presets.IntakeFromHPS);
            System.out.println("HPS time to: " + timeTo);

            focusCommand = drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.HumanPlayerStation,
                apriltagPoseAndId.getSecond());
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
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotStatus.setRobotState(RobotState.WaitingForCoralAtHPS);
    }
}
