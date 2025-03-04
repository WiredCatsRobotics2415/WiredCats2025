package frc.subsystems.slapdown;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class GenericSlapdown extends SubsystemBase {
    public abstract GenericSlapdownIO getIo();

    public abstract Command slapdown();

    public abstract Command stow();

    public abstract void setGoal(Angle goal);

    public abstract Angle getPivotAngle();

    public abstract Command toggleIntake();

    public abstract Command toggleOuttake();

    public abstract Command turnOffRollers();
}
