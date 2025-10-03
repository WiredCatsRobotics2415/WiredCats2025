package frc.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commands.ReefPresetTo.Level;
import frc.constants.Controls.AlignmentProfiles;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.utils.tuning.TuneableNumber;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AlignToReef extends Command {
    public static enum Side {
        Left, Right, Center
    }

    public static enum Face {
        AB, CD, EF, GH, IJ, KL
    }

    @Getter private static Pose2d lastApriltagAlignedTo;
    @Getter private static Pose2d lastAlignment;
    @Getter private static int lastApriltagIdAlignedTo;

    @Getter private static final TuneableNumber LeftOffset = new TuneableNumber(7.1, "AlignToReef/LeftOffset");
    @Getter private static final TuneableNumber RightOffset = new TuneableNumber(2.4, "AlignToReef/RightOffset");
    @Getter private static final TuneableNumber DriveTolerance = new TuneableNumber(1, "AlignToReef/DriveTolerance");
    @Getter private static final TuneableNumber LeftAlignRotation = new TuneableNumber(5.5,
        "AlignToReef/LeftAlignRotation");
    @Getter private static final TuneableNumber RightAlignRotation = new TuneableNumber(1,
        "AlignToReef/RightAlignRotation");
    @Getter private static Side lastSetSide;

    private TuneableNumber constructorGoalDriveOffset;
    private Face constructorFace;
    private boolean useProfiles;

    private TuneableNumber goalDriveOffset;
    private Side side;

    private Command driveCommand;
    private Command focusCommand;

    public AlignToReef(Side side, Face face, TuneableNumber driveOffset) {
        this.side = side;
        this.constructorFace = face;
        this.constructorGoalDriveOffset = driveOffset;
    }

    public AlignToReef(Side side, TuneableNumber driveOffset) {
        this.side = side;
        this.goalDriveOffset = driveOffset;
    }

    public AlignToReef(Side side, Face face) {
        this.side = side;
        this.constructorFace = face;
    }

    public AlignToReef(Side side, boolean useProfiles) {
        this.useProfiles = useProfiles;
        this.side = side;
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

        if (constructorFace == null) {
            System.out.println("Finding closest reef face");
            Pair<Pose2d, Integer> apriltagPoseAndId = AlignmentHelpers.findNearestApriltag(
                ReefMeasurements.reefApriltagsAlphabetic, ReefMeasurements.reefRedApriltags,
                ReefMeasurements.reefBlueApriltags, ReefMeasurements.reefIds, LimelightsForElements.Reef);
            apriltagId = apriltagPoseAndId.getSecond();
            apriltagPose = apriltagPoseAndId.getFirst();
        } else {
            List<Pose2d> poses = ReefMeasurements.reefApriltagsAlphabetic.get();
            int[] ids = ReefMeasurements.reefIds.get();
            System.out.println("using face " + constructorFace.toString());
            switch (constructorFace) {
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
        lastApriltagIdAlignedTo = apriltagId;
        Distance leftOffsetMeters = AlignToReef.LeftOffset.distance(),
            rightOffsetMeters = AlignToReef.RightOffset.distance();
        if (constructorGoalDriveOffset == null) {
            switch (ReefPresetTo.getLastLevelSet()) {
                case L1:
                    System.out.println("L1 set");
                    goalDriveOffset = Presets.Level1DriveOffset;
                    break;
                case L2:
                    System.out.println("L2 set");
                    goalDriveOffset = Presets.Level2DriveOffset;
                    break;
                case L2Scoring:
                    System.out.println("L2 scoring set");
                    goalDriveOffset = Presets.Level2DriveOffset;
                    break;
                case L3:
                    System.out.println("L3 set");
                    goalDriveOffset = Presets.Level3DriveOffset;
                    break;
                case L4:
                    System.out.println("L4 set");
                    goalDriveOffset = Presets.Level4DriveOffset;
                    break;
                default:
                    System.out.println("L1 set default");
                    goalDriveOffset = Presets.Level1DriveOffset;
                    break;
            }
        } else {
            goalDriveOffset = constructorGoalDriveOffset;
        }
        System.out.println("goal drive offset (in): " + goalDriveOffset.get());
        Transform2d leftOffset = new Transform2d(
            AlignmentHelpers.CenterToBumper.plus(goalDriveOffset.distance()).times(-1).in(Meters),
            leftOffsetMeters.in(Meters), Rotation2d.kZero.plus(LeftAlignRotation.rotation()));
        Transform2d rightOffset = new Transform2d(
            AlignmentHelpers.CenterToBumper.plus(goalDriveOffset.distance()).times(-1).in(Meters),
            -rightOffsetMeters.in(Meters), Rotation2d.kZero.plus(RightAlignRotation.rotation()));
        Transform2d offset;
        switch (side) {
            case Left:
                System.out.println("align to reef using left offset");
                offset = leftOffset;
                break;
            case Center:
                offset = new Transform2d(
                    AlignmentHelpers.CenterToBumper.plus(goalDriveOffset.distance()).times(-1).in(Meters), 0,
                    Rotation2d.kZero);
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

        if (useProfiles) {
            System.out.println("USING ALIGNMENT");
            // offset = ReefPresetTo.getLastLevelSet().equals(Level.L4) ? constructorL4Alignment
            // : constructorGeneralAlignment;
            if (side.equals(Side.Left)) {
                if (ReefPresetTo.getLastLevelSet().equals(Level.L4)) {
                    offset = AlignmentProfiles.LeftAlignmentL4;
                } else {
                    offset = AlignmentProfiles.LeftAlignmentL2L3;
                }
            } else {
                if (ReefPresetTo.getLastLevelSet().equals(Level.L4)) {
                    offset = AlignmentProfiles.RightAlignmentL4;
                } else {
                    offset = AlignmentProfiles.RightAlignmentL2L3;
                }
            }
        }
        lastSetSide = side;

        Pose2d driveTo = apriltagPose.plus(offset);
        lastAlignment = driveTo;
        System.out.println("offset distance: " + driveTo.getTranslation().getDistance(offset.getTranslation()));
        driveCommand = AlignmentHelpers.drive.driveTo(driveTo, DriveTolerance.meters());
        driveCommand.schedule();

        focusCommand = AlignmentHelpers.drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef, apriltagId);
        focusCommand.schedule();
    }

    @Override
    public void execute() {
        Logger.recordOutput("AlignToReef/SelectedApriltagPose", lastApriltagAlignedTo);
        Logger.recordOutput("AlignToReef/LevelSelection", ReefPresetTo.getLastLevelSet());
    }

    @Override
    public boolean isFinished() { return driveCommand.isFinished(); }

    @Override
    public void end(boolean interrupted) {
        focusCommand.cancel();
        driveCommand.cancel();
        System.out.println("alignment to reef is DONE, interrupted: " + interrupted);
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
