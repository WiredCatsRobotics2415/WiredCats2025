package frc.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls;
import frc.subsystems.drive.CommandSwerveDrivetrain;

public class MinorDriveAdjuster extends Command {
    public static enum Direction {
        Forward, Backward, Left, Right
    }

    private CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
    private RobotCentric request = new RobotCentric();

    public MinorDriveAdjuster(Direction direction) {
        switch (direction) {
            case Forward:
                request.VelocityX = Controls.MinorAdjustmentSpeedMeterS;
                break;
            case Backward:
                request.VelocityX = -Controls.MinorAdjustmentSpeedMeterS;
                break;
            case Left:
                request.VelocityY = Controls.MinorAdjustmentSpeedMeterS;
                break;
            case Right:
                request.VelocityY = -Controls.MinorAdjustmentSpeedMeterS;
                break;
            default:
                break;
        }
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setControl(request);
    }
}
