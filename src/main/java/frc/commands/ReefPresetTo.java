package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Controls.Presets;
import frc.robot.RobotStatus;
import frc.robot.RobotStatus.RobotState;
import frc.subsystems.superstructure.SuperStructure;
import frc.subsystems.superstructure.TuneableSuperStructureState;
import lombok.Getter;

public class ReefPresetTo extends Command {
    public static enum Level {
        L1, L2, L3, L4,
    }

    @Getter private static Level lastLevelSet = Level.L1;
    private final static SuperStructure superStructure = SuperStructure.getInstance();

    private TuneableSuperStructureState superStructureState;
    private Command superStructureCommand;
    private Level thisLevel;

    public ReefPresetTo(Level reefLevel) {
        thisLevel = reefLevel;
        switch (reefLevel) {
            case L1:
                superStructureState = Presets.Level1;
                break;
            case L2:
                superStructureState = Presets.Level2;
                break;
            case L3:
                superStructureState = Presets.Level3;
                break;
            case L4:
                superStructureState = Presets.Level4;
                break;
            default:
                superStructureState = Presets.Level1;
                break;
        }
    }

    @Override
    public void initialize() {
        superStructureCommand = superStructure.beThereAsapNoEnd(superStructureState, false, false);
        superStructureCommand.schedule();
        lastLevelSet = thisLevel;

        RobotStatus.setRobotState(RobotState.AligningToScoreCoral);
    }

    @Override
    public boolean isFinished() { return superStructure.doneWithMovement() || superStructureCommand.isScheduled(); }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ReefPresetTo is finished, interrupted: " + interrupted);
        RobotStatus.setRobotState(RobotState.WaitingToScoreCoral);
    }
}
