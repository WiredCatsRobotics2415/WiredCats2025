package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.Controls.Presets;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.elevator.Elevator;
import frc.subsystems.superstructure.SuperStructure;

public class CommonCommands {
    private static SuperStructure superstructure = SuperStructure.getInstance();
    private static Elevator elevator = Elevator.getInstance();

    public static SequentialCommandGroup stowFromGroundIntake() {
        return new SequentialCommandGroup(superstructure.beThereAsap(Presets.BumpStow),
            Commands.waitUntil(elevator::atGoal), superstructure.beThereAsapAndEnd(Presets.Stow),
            RobotStatus.setRobotStateOnce(RobotState.Stow));
    }

    public static SequentialCommandGroup stowNormally() {
        return new SequentialCommandGroup(superstructure.beThereAsapAndEnd(Presets.Stow),
            RobotStatus.setRobotStateOnce(RobotState.Stow));
    }
}
