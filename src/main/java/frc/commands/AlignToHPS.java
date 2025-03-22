package frc.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.HumanPlayerStation;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.utils.tuning.TuneableNumber;
import java.util.List;

public class AlignToHPS extends Command {
    public static enum HPSSide {
        Left, Right
    }

    public static enum FieldSideRelativeToDriver {
        Left, Right
    }

    private static final Transform2d Offset = new Transform2d(AlignmentHelpers.CenterToBumper, Inches.of(0),
        Rotation2d.kZero);

    private static final TuneableNumber LeftOffset = new TuneableNumber(6, "AlignToHPS/LeftOffset");
    private static final TuneableNumber RightOffset = new TuneableNumber(6, "AlignToHPS/RightOffset");
    private static final TuneableNumber DriveTolerance = new TuneableNumber(3, "AlignToHPS/DriveTolerance");

    private HPSSide side;
    private FieldSideRelativeToDriver fieldSide;

    private Command driveCommand;
    private Command focusCommand;

    public AlignToHPS(HPSSide side, FieldSideRelativeToDriver fieldSide) {
        this.side = side;
        this.fieldSide = fieldSide;
    }

    public AlignToHPS(HPSSide side) {
        this.side = side;
    }

    @Override
    public void initialize() {
        Pose2d apriltagPose;
        Integer apriltagId;

        if (fieldSide == null) {
            System.out.println("Finding closest hps");
            Pair<Pose2d, Integer> apriltagPoseAndId = AlignmentHelpers.findNearestApriltag(
                HumanPlayerStation.hpsApriltags, HumanPlayerStation.hpsRedApriltags,
                HumanPlayerStation.hpsBlueApriltags, HumanPlayerStation.hpsIds,
                LimelightsForElements.HumanPlayerStation);
            apriltagId = apriltagPoseAndId.getSecond();
            apriltagPose = apriltagPoseAndId.getFirst();
        } else {
            List<Pose2d> poses = HumanPlayerStation.hpsApriltags.get();
            int[] ids = HumanPlayerStation.hpsIds.get();
            System.out.println("using side " + side.toString());
            switch (fieldSide) {
                case Left:
                    apriltagPose = poses.get(0);
                    apriltagId = ids[0];
                    break;
                case Right:
                    apriltagPose = poses.get(1);
                    apriltagId = ids[1];
                    break;
                default:
                    apriltagPose = poses.get(0);
                    apriltagId = ids[0];
                    break;
            }
        }
        Pose2d driveToPose = apriltagPose.plus(Offset).plus(new Transform2d(Presets.HPSDriveOffset.meters(),
            side == HPSSide.Left ? LeftOffset.meters() : RightOffset.meters(), Rotation2d.kZero));
        driveCommand = AlignmentHelpers.drive.driveTo(driveToPose, DriveTolerance.meters());
        driveCommand.schedule();

        focusCommand = AlignmentHelpers.drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, apriltagId);
        focusCommand.schedule();
    }

    @Override
    public boolean isFinished() { return driveCommand.isFinished(); }

    @Override
    public void end(boolean interrupted) {
        focusCommand.cancel();
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
