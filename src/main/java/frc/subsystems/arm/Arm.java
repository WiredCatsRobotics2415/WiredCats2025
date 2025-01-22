package frc.subsystems.arm;

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

    /** Sets the goal height. If goalInches is out of the physical range, it is not set. */
    public void setGoal(double goalDegrees) {
        if (goalDegrees > ArmConstants.MaxDegreesFront || goalDegrees < ArmConstants.MaxDegreesBack) return;
        this.goalDegrees = goalDegrees;
    }
}
