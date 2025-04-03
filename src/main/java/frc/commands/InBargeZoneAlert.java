package frc.commands;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.constants.Subsystems.VisionConstants;
import frc.subsystems.drive.CommandSwerveDrivetrain;
import frc.utils.AllianceDependent;
import frc.utils.driver.DriverFeedback;

public class InBargeZoneAlert extends Command {
    private final CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();

    private final AllianceDependent<Rectangle2d> bargeZone = new AllianceDependent<Rectangle2d>(
        new Rectangle2d(new Translation2d(7.125, 7.3), new Translation2d(7.6, 5)),
        new Rectangle2d(new Translation2d(10.4, 3.2), new Translation2d(9.85, 0.64)));

    private final Command flashCommand = Commands.parallel(
        DriverFeedback.keepLimelightOnUntilUpdated(VisionConstants.BackCenterName),
        DriverFeedback.keepLimelightOnUntilUpdated(VisionConstants.FrontLeftName),
        DriverFeedback.keepLimelightOnUntilUpdated(VisionConstants.FrontRightName));

    public InBargeZoneAlert() {

    }

    @Override
    public void execute() {
        if (bargeZone.get().contains(drive.getState().Pose.getTranslation())) {
            if (!flashCommand.isScheduled()) flashCommand.schedule();
        } else {
            if (flashCommand.isScheduled()) flashCommand.cancel();
        }
    }
}
