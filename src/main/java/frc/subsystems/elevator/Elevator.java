package frc.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Subsystems.ElevatorConstants;
import lombok.Getter;

public class Elevator extends SubsystemBase {
    @Getter private double goalInches = 0.0;

    private static Elevator instance;

    private Elevator() {

    }

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalInches) {
        if (goalInches > ElevatorConstants.MaxHeight || goalInches < 0) return;
        this.goalInches = goalInches;
    }
}
