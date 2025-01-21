package frc.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.constants.Subsystems.ArmConstants;
import lombok.Getter;

public class Arm {
    @Getter private double goalDegrees = 0.0;

    private static Arm instance;

    private Arm() {

    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    /** Change goal by changeBy. Negatives work, bounds are checked. Intended for manual control. */
    public Command changeGoal(double changeBy) {
        return new RepeatCommand(new InstantCommand(() -> {
            this.setGoal(this.getGoalDegrees() + changeBy);
        }));
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalDegrees) {
        if (goalDegrees > ArmConstants.MaxDegreesFront || goalDegrees < ArmConstants.MaxDegreesBack) return;
        this.goalDegrees = goalDegrees;
    }
}
