package frc.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.utils.tuning.TuneableNumber;
import java.util.List;
import lombok.Getter;

public class AlignToReef extends Command {
    public static enum Side {
        Left, Right, Center
    }

    public static enum Face {
        AB, CD, EF, GH, IJ, KL
    }

    @Getter private static Pose2d lastApriltagAlignedTo;

    @Getter private static final TuneableNumber LeftOffset = new TuneableNumber(11, "AlignToReef/LeftOffset");
    @Getter private static final TuneableNumber RightOffset = new TuneableNumber(5, "AlignToReef/RightOffset");
    private static final TuneableNumber DriveTolerance = new TuneableNumber(0.5, "AlignToReef/DriveTolerance");

    private TuneableNumber goalDriveOffset;
    private Side side;
    private Face face;

    private Command driveCommand;
    private Command focusCommand;

    public AlignToReef(Side side, Face face, TuneableNumber driveOffset) {
        this.side = side;
        this.face = face;
        this.goalDriveOffset = driveOffset;
    }

    public AlignToReef(Side side, TuneableNumber driveOffset) {
        this.side = side;
        this.goalDriveOffset = driveOffset;
    }

    public AlignToReef(Side side, Face face) {
        this.side = side;
        this.face = face;
    }

    /**
     * Note when using this constructor: the driveOffset (ie. how far back the robot is from the chosen apriltag) is automatically determined from the last set REEF preset and does NOT consider dealgae preset
     */
    public AlignToReef(Side side) {
        this.side = side;
    }

    @Override
    public void initialize() {
        Pose2d apriltagPose;
        Integer apriltagId;

        if (face == null) {
            System.out.println("Finding closest reef face");
            Pair<Pose2d, Integer> apriltagPoseAndId = AlignmentHelpers.findNearestApriltag(
                ReefMeasurements.reefApriltagsAlphabetic, ReefMeasurements.reefRedApriltags,
                ReefMeasurements.reefBlueApriltags, ReefMeasurements.reefIds, LimelightsForElements.Reef);
            apriltagId = apriltagPoseAndId.getSecond();
            apriltagPose = apriltagPoseAndId.getFirst();
        } else {
            List<Pose2d> poses = ReefMeasurements.reefApriltagsAlphabetic.get();
            int[] ids = ReefMeasurements.reefIds.get();
            System.out.println("using face " + face.toString());
            switch (face) {
                case AB:
                    apriltagPose = poses.get(0);
                    apriltagId = ids[0];
                    break;
                case CD:
                    apriltagPose = poses.get(1);
                    apriltagId = ids[1];
                    break;
                case EF:
                    apriltagPose = poses.get(2);
                    apriltagId = ids[2];
                    break;
                case GH:
                    apriltagPose = poses.get(3);
                    apriltagId = ids[3];
                    break;
                case IJ:
                    apriltagPose = poses.get(4);
                    apriltagId = ids[4];
                    break;
                case KL:
                    apriltagPose = poses.get(5);
                    apriltagId = ids[5];
                    break;
                default:
                    apriltagPose = poses.get(0);
                    apriltagId = ids[0];
                    break;
            }
        }
        lastApriltagAlignedTo = apriltagPose;
        Distance leftOffsetMeters = AlignToReef.LeftOffset.distance(),
            rightOffsetMeters = AlignToReef.RightOffset.distance();
        if (goalDriveOffset == null) {
            switch (ReefPresetTo.getLastLevelSet()) {
                case L1:
                    goalDriveOffset = Presets.Level1DriveOffset;
                    leftOffsetMeters = Presets.Level1OffsetLeft.distance();
                    rightOffsetMeters = Presets.Level1OffsetRight.distance();
                    break;
                case L2:
                    goalDriveOffset = Presets.Level2DriveOffset;
                    leftOffsetMeters = Presets.Level2OffsetLeft.distance();
                    rightOffsetMeters = Presets.Level2OffsetRight.distance();
                    break;
                case L3:
                    goalDriveOffset = Presets.Level3DriveOffset;
                    leftOffsetMeters = Presets.Level3OffsetLeft.distance();
                    rightOffsetMeters = Presets.Level3OffsetRight.distance();
                    break;
                case L4:
                    goalDriveOffset = Presets.Level4DriveOffset;
                    leftOffsetMeters = Presets.Level4OffsetLeft.distance();
                    rightOffsetMeters = Presets.Level4OffsetRight.distance();
                    break;
                default:
                    goalDriveOffset = Presets.Level1DriveOffset;
                    leftOffsetMeters = Presets.Level1OffsetLeft.distance();
                    rightOffsetMeters = Presets.Level1OffsetRight.distance();
                    break;
            }
        }
        System.out.println("goal drive offset (in): " + goalDriveOffset.get());
        System.out.println(AlignmentHelpers.CenterToBumper.in(Inches));
        Transform2d leftOffset = new Transform2d(
            AlignmentHelpers.CenterToBumper.plus(goalDriveOffset.distance()).times(-1).in(Meters),
            leftOffsetMeters.in(Meters), Rotation2d.kZero);
        Transform2d rightOffset = new Transform2d(
            AlignmentHelpers.CenterToBumper.plus(goalDriveOffset.distance()).times(-1).in(Meters),
            -rightOffsetMeters.in(Meters), Rotation2d.kZero);
        Transform2d offset;
        switch (side) {
            case Left:
                System.out.println("align to reef using left offset");
                offset = leftOffset;
                break;
            case Center:
                offset = Transform2d.kZero;
                break;
            case Right:
                System.out.println("align to reef using right offset");
                offset = rightOffset;
                break;
            default:
                System.out.println("align to reef using left offset (default case)");
                offset = leftOffset;
                break;
        }
        System.out.println("offset x: " + offset.getX() + ", y: " + offset.getY());

        Pose2d driveTo = apriltagPose.plus(offset);
        driveCommand = AlignmentHelpers.drive.driveTo(driveTo, DriveTolerance.meters());
        driveCommand.schedule();

        focusCommand = AlignmentHelpers.drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, apriltagId);
        focusCommand.schedule();
    }

    @Override
    public boolean isFinished() { return driveCommand.isFinished(); }

    @Override
    public void end(boolean interrupted) {
        focusCommand.cancel();
        driveCommand.cancel();
        System.out.println("alignment to reef is DONE");
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
