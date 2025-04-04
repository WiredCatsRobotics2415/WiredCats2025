package frc.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Subsystems.VisionConstants.LimelightsForElements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.utils.tuning.TuneableNumber;

public class Dealgae extends Command {
    private static TuneableNumber amountToMoveForward = new TuneableNumber(4, "Dealgae/AmountToMoveForward");
    private static TuneableNumber driveTolerance = new TuneableNumber(1, "Dealgae/DriveTolerance");

    Command driveCommand;
    Command focusCommand;

    @Override
    public void initialize() {
        Pose2d driveTo = AlignmentHelpers.drive.getState().Pose
            .plus(new Transform2d(amountToMoveForward.get(), 0, Rotation2d.kZero));
        driveCommand = AlignmentHelpers.drive.driveTo(driveTo, driveTolerance.meters());
        driveCommand.schedule();

        focusCommand = AlignmentHelpers.drive.focusOnTagWhenSeenTemporarily(LimelightsForElements.Reef,
            AlignToReef.getLastApriltagIdAlignedTo());
        focusCommand.schedule();
    }

    @Override
    public boolean isFinished() { return driveCommand.isFinished(); }

    @Override
    public void end(boolean interrupted) {
        focusCommand.cancel();
        driveCommand.cancel();
        System.out.println("dealgae is DONE");
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
