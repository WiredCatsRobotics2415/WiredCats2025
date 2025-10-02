package frc.utils.driver;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.constants.Controls;
import frc.constants.Subsystems.VisionConstants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.utils.LimelightHelpers;

public class DriverFeedback {
    private static XboxController currentRumbleableController = null;

    public static void flashLimelight(String limelight) {
        LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.BackCenterName);
    }

    public static void turnOffLimelight(String limelight) {
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.BackCenterName);
    }

    public static Command blinkBackLLInConfirmation() {
        return new InstantCommand(() -> flashLimelight(VisionConstants.BackCenterName)).andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> flashLimelight(VisionConstants.BackCenterName)));
    }

    public static Command keepLimelightOnUntilUpdated(String limelight) {
        return Commands.runOnce(() -> {
            LimelightHelpers.setLEDMode_ForceOn(limelight);
        }).finallyDo(() -> {
            LimelightHelpers.setLEDMode_ForceOff(limelight);
        });
    }

    private static boolean controllerExists() {
        if (currentRumbleableController == null) {
            OI oi = RobotContainer.getInstance().getOi();
            if (oi == null) return false;
            currentRumbleableController = oi.getHIDOfController();
        }
        return true;
    }

    public static void rumbleSoft() {
        if (!controllerExists()) return;
        currentRumbleableController.setRumble(RumbleType.kBothRumble, Controls.RumbleSoftValue);
    }

    public static void noRumble() {
        if (!controllerExists()) return;
        currentRumbleableController.setRumble(RumbleType.kBothRumble, 0);
    }
}
