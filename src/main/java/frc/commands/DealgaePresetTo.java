package frc.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.constants.Measurements.ReefMeasurements;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.superstructure.SuperStructure;
import frc.utils.AllianceDependent;

public class DealgaePresetTo extends Command {
    boolean autoSelectDealgae = true;
    boolean manualSelectGettingTop = true;

    private final static SuperStructure superStructure = SuperStructure.getInstance();

    private Command superStructureCommand;

    private boolean algaeOnTopForPose(Pose2d apriltagPose) {
        if (AllianceDependent.isCurrentlyBlue()) {
            return ReefMeasurements.ReefAlgaeOnTopAlphabeticOrder
                .get(ReefMeasurements.reefBlueApriltags.indexOf(apriltagPose));
        } else {
            return ReefMeasurements.ReefAlgaeOnTopAlphabeticOrder
                .get(ReefMeasurements.reefRedApriltags.indexOf(apriltagPose));
        }
    }

    public DealgaePresetTo() {
        autoSelectDealgae = true;
    }

    /**
     * Manually select whether to dealgae top or bottom. Disables driveTo.
     */
    public DealgaePresetTo(boolean dealgaeTop) {
        autoSelectDealgae = false;
        manualSelectGettingTop = dealgaeTop;
    }

    @Override
    public void initialize() {
        if (autoSelectDealgae) {
            boolean algaeOnTop = algaeOnTopForPose(AlignToReef.getLastApriltagAlignedTo());

            superStructureCommand = SuperStructure.getInstance()
                .beThereAsapNoEnd(algaeOnTop ? Presets.TopDeAlgae : Presets.BottomDeAlgae);
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
    public boolean isFinished() { return superStructure.doneWithMovement(); }

    @Override
    public void end(boolean interrupted) {
        RobotStatus.setRobotState(RobotState.WaitingToDeAlgae);
    }
}
